import os
import glob
import time
import requests
import xml.etree.ElementTree as ET
from bs4 import BeautifulSoup
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from tqdm import tqdm
from dotenv import load_dotenv
from langchain_text_splitters import RecursiveCharacterTextSplitter

# Load keys
load_dotenv()
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_KEY = os.getenv("QDRANT_API_KEY")
COHERE_KEY = os.getenv("COHERE_API_KEY")

if not QDRANT_URL or not QDRANT_KEY or not COHERE_KEY:
    print("❌ Error: Please set QDRANT_URL, QDRANT_API_KEY, and COHERE_API_KEY in .env")
    exit(1)

# Initialize Clients
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_KEY)
co = cohere.Client(COHERE_KEY)

COLLECTION_NAME = "textbook_knowledge"
EMBED_MODEL = "embed-english-v3.0"
SITEMAP_URL = "https://devhasnainraza.github.io/Cortex-H1/sitemap.xml"

def get_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """Fetches sitemap and parses it to extract all URLs."""
    print(f"Fetching sitemap from {sitemap_url}...")
    response = requests.get(sitemap_url)
    response.raise_for_status()
    
    root = ET.fromstring(response.content)
    urls = [elem.text for elem in root.findall(".//{http://www.sitemaps.org/schemas/sitemap/0.9}loc")]
    print(f"Found {len(urls)} URLs in the sitemap.")
    return urls

def scrape_and_clean_text(url: str) -> str:
    """Scrapes a URL and cleans the HTML to get the main text content."""
    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'lxml')
        
        # This selector is specific to Docusaurus's default theme.
        main_content = soup.find('article')
        
        if main_content:
            return main_content.get_text(separator=' ', strip=True)
        else:
            # Fallback for pages that might not have an <article> tag
            return soup.get_text(separator=' ', strip=True)
    except requests.RequestException as e:
        print(f"⚠️ Could not fetch {url}: {e}")
        return ""

def get_embedding_batch(texts: List[str]) -> List[List[float]]:
    """Generates embeddings for a batch of texts using Cohere with retry logic."""
    retries = 10
    delay = 5
    for i in range(retries):
        try:
            response = co.embed(
                texts=texts,
                model=EMBED_MODEL,
                input_type="search_document"
            )
            return response.embeddings
        except Exception as e:
            if "429" in str(e): # Check for rate limit error
                print(f"    Rate limit hit, retrying in {delay} seconds...")
                time.sleep(delay)
                delay *= 2 # Exponential backoff
            else:
                raise e
    raise Exception("Failed to get embeddings after multiple retries.")

def chunk_text(text: str, chunk_size=1000, chunk_overlap=100) -> List[str]:
    """Splits text into chunks."""
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap,
        length_function=len,
    )
    return text_splitter.split_text(text)

def ingest_sitemap():
    print(f"🚀 Starting Ingestion to Qdrant collection: {COLLECTION_NAME}")
    
    # 1. Recreate Collection
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )

    # 2. Fetch and process URLs from sitemap
    urls = get_urls_from_sitemap(SITEMAP_URL)
    total_segments = 0

    for url in tqdm(urls, desc="Processing URLs"):
        print(f"📄 Processing {url}...")
        text_content = scrape_and_clean_text(url)
        if not text_content:
            continue

        chunks = chunk_text(text_content)
        
        # Batch processing
        batch_size = 96
        for i in range(0, len(chunks), batch_size):
            batch_chunks = chunks[i:i + batch_size]
            print(f"  - Processing batch {i//batch_size + 1}/{(len(chunks) + batch_size - 1)//batch_size}...")

            try:
                embeddings = get_embedding_batch(batch_chunks)
                
                points_to_upsert = [
                    models.Point(
                        id=f"{url}-{i+j}",
                        vector=embedding,
                        payload={"text": chunk, "source": url}
                    )
                    for j, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings))
                ]
                
                qdrant.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points_to_upsert
                )
                total_segments += len(points_to_upsert)
                print(f"    ✅ Upserted {len(points_to_upsert)} points.")

            except Exception as e:
                print(f"⚠️ Error processing batch for {url}: {e}")

    print(f"🎉 Ingestion Complete! Total segments: {total_segments}")

if __name__ == "__main__":
    ingest_sitemap()
