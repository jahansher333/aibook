#!/usr/bin/env python3
"""
Simple ingestion script for Windows - ingests Chapter 1 into Qdrant
"""

import os
import sys
import re
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import tiktoken
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

encoding = tiktoken.get_encoding("cl100k_base")

def parse_chapter(file_path):
    """Parse chapter file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter
    frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)$', content, re.DOTALL)
    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        main_content = frontmatter_match.group(2)

        # Extract metadata
        chapter_id_match = re.search(r'id:\s*([^\n]+)', frontmatter)
        title_match = re.search(r'title:\s*["\']?([^"\'\n]+)["\']?', frontmatter)

        chapter_id = chapter_id_match.group(1).strip() if chapter_id_match else Path(file_path).stem
        chapter_title = title_match.group(1).strip() if title_match else chapter_id

        # Remove code blocks
        content_no_code = re.sub(r'```[\s\S]*?```', '', main_content)

        return {
            'chapter_id': chapter_id,
            'chapter_title': chapter_title,
            'content': content_no_code.strip()
        }

    return None

def chunk_text(text, max_tokens=512, overlap=50):
    """Chunk text into overlapping segments"""
    tokens = encoding.encode(text)
    chunks = []

    for i in range(0, len(tokens), max_tokens - overlap):
        chunk_tokens = tokens[i:i + max_tokens]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)

    return chunks

def main():
    collection_name = "physical_ai_book"
    chapter_file = "../docusaurus/docs/ch01-physical-ai-intro/ch01.md"

    print(f"Ingesting: {chapter_file}")

    # Parse chapter
    chapter = parse_chapter(chapter_file)
    if not chapter:
        print("ERROR: Could not parse chapter")
        return

    print(f"Chapter: {chapter['chapter_title']}")

    # Chunk content
    chunks = chunk_text(chapter['content'])
    print(f"Created {len(chunks)} chunks")

    # Generate embeddings
    print("Generating embeddings...")
    response = co.embed(
        texts=chunks,
        model="embed-english-v3.0",
        input_type="search_document",
        embedding_types=["float"]
    )
    embeddings = response.embeddings.float

    print(f"Got embeddings")

    # Create points
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        points.append(PointStruct(
            id=i,
            vector=embedding,
            payload={
                "content": chunk,
                "chapter_id": chapter['chapter_id'],
                "chapter_title": chapter['chapter_title'],
                "chunk_index": i
            }
        ))

    # Upsert to Qdrant
    print(f"Uploading {len(points)} points to Qdrant...")
    qdrant.upsert(
        collection_name=collection_name,
        points=points
    )

    print("[SUCCESS] Ingestion complete!")
    print(f"Total chunks: {len(chunks)}")

if __name__ == "__main__":
    main()
