"""
RAG Ingestion Script
Processes markdown files from docs/ and ingests them into Qdrant Cloud
Uses OpenAI embeddings for vector generation
"""
import os
import glob
import hashlib
from pathlib import Path
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
from langchain_text_splitters import MarkdownHeaderTextSplitter

load_dotenv()

# Configuration
DOCS_DIR = os.getenv("DOCS_DIR", "docs")
COLLECTION_NAME = "textbook_knowledge"
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable is required")
if not QDRANT_API_KEY:
    print("Warning: QDRANT_API_KEY not set, using local Qdrant")

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)

try:
    if QDRANT_URL and QDRANT_API_KEY:
        qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print(f"Connected to Qdrant Cloud: {QDRANT_URL}")
    else:
        print("Warning: Using local Qdrant instance")
        qdrant = QdrantClient("http://localhost:6333")
except Exception as e:
    print(f"Qdrant Connection Error: {e}")
    raise

# Ensure collection exists
try:
    collection_info = qdrant.get_collection(COLLECTION_NAME)
    print(f"Collection '{COLLECTION_NAME}' exists with {collection_info.points_count} points")
except Exception:
    # Create collection with OpenAI embedding size (1536 for text-embedding-3-small)
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=1536,
            distance=models.Distance.COSINE
        ),
    )
    print(f"Created collection '{COLLECTION_NAME}'")


def generate_point_id(text: str, source: str) -> int:
    """Generate a deterministic point ID from text and source"""
    combined = f"{source}:{text}"
    return int(hashlib.md5(combined.encode()).hexdigest()[:15], 16) % (10**16)


def ingest_docs():
    """Main ingestion function"""
    print(f"Scanning {DOCS_DIR} for markdown files...")
    
    # Find all markdown files
    docs_path = Path(DOCS_DIR)
    if not docs_path.exists():
        print(f"Error: {DOCS_DIR} directory does not exist")
        return
    
    files = list(docs_path.rglob("*.md"))
    print(f"Found {len(files)} markdown files")
    
    # Configure text splitter
    headers_to_split_on = [
        ("#", "Header 1"),
        ("##", "Header 2"),
        ("###", "Header 3"),
        ("####", "Header 4"),
    ]
    markdown_splitter = MarkdownHeaderTextSplitter(
        headers_to_split_on=headers_to_split_on
    )
    
    all_chunks = []
    for file_path in files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
            # Split by headers
            splits = markdown_splitter.split_text(content)
            
            for split in splits:
                # Combine metadata headers with source
                metadata = {
                    "source": str(file_path.relative_to(docs_path)),
                    "source_path": str(file_path),
                    **split.metadata
                }
                
                chunk_data = {
                    "text": split.page_content,
                    "metadata": metadata
                }
                all_chunks.append(chunk_data)
                
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            continue
    
    print(f"Generated {len(all_chunks)} text chunks")
    
    # Batch process chunks for embedding
    batch_size = 100
    total_ingested = 0
    
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i + batch_size]
        print(f"Processing batch {i//batch_size + 1}/{(len(all_chunks) + batch_size - 1)//batch_size}...")
        
        # Extract texts for embedding
        texts = [chunk["text"] for chunk in batch]
        
        # Generate embeddings using OpenAI
        try:
            embedding_response = openai_client.embeddings.create(
                model="text-embedding-3-small",
                input=texts
            )
            
            # Prepare points for Qdrant
            points = []
            for idx, chunk in enumerate(batch):
                point_id = generate_point_id(
                    chunk["text"],
                    chunk["metadata"]["source"]
                )
                
                points.append(
                    models.PointStruct(
                        id=point_id,
                        vector=embedding_response.data[idx].embedding,
                        payload={
                            "text": chunk["text"],
                            **chunk["metadata"]
                        }
                    )
                )
            
            # Upsert to Qdrant
            qdrant.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            
            total_ingested += len(batch)
            print(f"Ingested {total_ingested}/{len(all_chunks)} chunks")
            
        except Exception as e:
            print(f"Error processing batch: {e}")
            continue
    
    print(f"\n✅ Ingestion complete! Total chunks ingested: {total_ingested}")
    
    # Get final collection info
    try:
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print(f"Collection now has {collection_info.points_count} total points")
    except Exception as e:
        print(f"Could not retrieve collection info: {e}")


if __name__ == "__main__":
    ingest_docs()
