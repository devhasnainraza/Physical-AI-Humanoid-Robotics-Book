import os
import glob
from langchain_community.vectorstores import Qdrant
from langchain_openai import OpenAIEmbeddings
from langchain_text_splitters import MarkdownHeaderTextSplitter

# Configuration
DOCS_DIR = "docs"
COLLECTION_NAME = "textbook_knowledge"
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

def ingest_docs():
    print(f"Scanning {DOCS_DIR}...")
    files = glob.glob(f"{DOCS_DIR}/**/*.md", recursive=True)
    
    headers_to_split_on = [
        ("#", "Header 1"),
        ("##", "Header 2"),
        ("###", "Header 3"),
    ]
    markdown_splitter = MarkdownHeaderTextSplitter(headers_to_split_on=headers_to_split_on)
    
    all_splits = []
    for file_path in files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            splits = markdown_splitter.split_text(content)
            for split in splits:
                split.metadata["source"] = file_path
            all_splits.extend(splits)
            
    print(f"Generated {len(all_splits)} chunks.")
    
    # In a real run, we would upsert these to Qdrant using OpenAI embeddings
    # qdrant = Qdrant.from_documents(
    #     all_splits,
    #     OpenAIEmbeddings(),
    #     url=QDRANT_URL,
    #     collection_name=COLLECTION_NAME,
    # )
    print("Ingestion complete (Simulated).")

if __name__ == "__main__":
    ingest_docs()
