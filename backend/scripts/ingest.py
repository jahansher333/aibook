#!/usr/bin/env python3
"""
Ingest Physical AI textbook chapters into Qdrant vector database.

Usage:
    python scripts/ingest.py --chapters ../docusaurus/docs/ch*/*.md
"""

import os
import sys
import re
import glob
import argparse
from pathlib import Path
import tiktoken
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv
from tqdm import tqdm

# Load environment variables
load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Tokenizer for chunking
encoding = tiktoken.get_encoding("cl100k_base")

def parse_mdx(file_path):
    """Parse MDX file and extract metadata and content"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter
    frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)$', content, re.DOTALL)
    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        main_content = frontmatter_match.group(2)

        # Extract chapter_id and title from frontmatter
        chapter_id_match = re.search(r'id:\s*([^\n]+)', frontmatter)
        title_match = re.search(r'title:\s*["\']?([^"\'\n]+)["\']?', frontmatter)

        chapter_id = chapter_id_match.group(1).strip() if chapter_id_match else Path(file_path).stem
        chapter_title = title_match.group(1).strip() if title_match else "Unknown Chapter"
    else:
        main_content = content
        chapter_id = Path(file_path).stem
        chapter_title = "Unknown Chapter"

    # Remove code blocks and mermaid diagrams
    main_content = re.sub(r'```[\s\S]*?```', '', main_content)

    # Extract section titles (## headings)
    sections = re.split(r'\n(#{2,3}\s+[^\n]+)', main_content)

    return {
        'chapter_id': chapter_id,
        'chapter_title': chapter_title,
        'content': main_content,
        'sections': sections
    }

def chunk_text(text, max_tokens=512, overlap=50):
    """Split text into chunks of max_tokens with overlap"""
    tokens = encoding.encode(text)
    chunks = []

    start = 0
    while start < len(tokens):
        end = start + max_tokens
        chunk_tokens = tokens[start:end]
        chunk_text = encoding.decode(chunk_tokens)

        # Try to end at sentence boundary
        if end < len(tokens):
            last_period = chunk_text.rfind('.')
            last_newline = chunk_text.rfind('\n')
            boundary = max(last_period, last_newline)
            if boundary > len(chunk_text) * 0.7:  # Only if boundary is in last 30%
                chunk_text = chunk_text[:boundary + 1]

        chunks.append(chunk_text.strip())
        start = end - overlap

    return chunks

def ingest_chapters(chapter_paths, collection_name="physical_ai_book"):
    """Ingest chapters into Qdrant"""

    # Create collection if it doesn't exist
    try:
        qdrant.get_collection(collection_name)
        print(f"✓ Collection '{collection_name}' exists")
    except:
        print(f"Creating collection '{collection_name}'...")
        qdrant.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,  # Cohere embed-english-v3.0
                distance=Distance.COSINE
            )
        )
        print("✓ Collection created")

    # Process all chapters
    all_chunks = []
    all_embeddings = []
    chunk_id = 0

    for chapter_path in tqdm(chapter_paths, desc="Processing chapters"):
        chapter_data = parse_mdx(chapter_path)

        # Chunk the content
        chunks = chunk_text(chapter_data['content'])

        for i, chunk in enumerate(chunks):
            all_chunks.append({
                'id': f"{chapter_data['chapter_id']}-chunk{i}",
                'chapter_id': chapter_data['chapter_id'],
                'chapter_title': chapter_data['chapter_title'],
                'content': chunk,
                'chunk_index': i
            })

            # Batch embeddings (Cohere API limit: 96 texts per call)
            if len(all_chunks) % 50 == 0 or chapter_path == chapter_paths[-1]:
                print(f"\nGenerating embeddings for {len(all_chunks)} chunks...")

                # Get texts for embedding
                texts_to_embed = [c['content'] for c in all_chunks[len(all_embeddings):]]

                # Generate embeddings
                response = co.embed(
                    texts=texts_to_embed,
                    model="embed-english-v3.0",
                    input_type="search_document"
                )
                all_embeddings.extend(response.embeddings)

    # Upsert to Qdrant
    print(f"\nUpserting {len(all_chunks)} chunks to Qdrant...")
    points = []
    for i, chunk in enumerate(all_chunks):
        points.append(
            PointStruct(
                id=i,
                vector=all_embeddings[i],
                payload={
                    'chunk_id': chunk['id'],
                    'chapter_id': chunk['chapter_id'],
                    'chapter_title': chunk['chapter_title'],
                    'content': chunk['content'],
                    'chunk_index': chunk['chunk_index']
                }
            )
        )

    # Batch upsert
    batch_size = 100
    for i in range(0, len(points), batch_size):
        qdrant.upsert(
            collection_name=collection_name,
            points=points[i:i+batch_size]
        )

    print(f"✓ Ingestion complete: {len(all_chunks)} chunks")
    print(f"✓ Total tokens: ~{sum(len(encoding.encode(c['content'])) for c in all_chunks)}")

    # Verify
    collection_info = qdrant.get_collection(collection_name)
    print(f"✓ Qdrant collection size: {collection_info.points_count} points")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ingest textbook chapters into Qdrant")
    parser.add_argument(
        "--chapters",
        type=str,
        required=True,
        help="Glob pattern for chapter MDX files (e.g., ../docusaurus/docs/ch*/*.md)"
    )
    parser.add_argument(
        "--collection",
        type=str,
        default="physical_ai_book",
        help="Qdrant collection name"
    )

    args = parser.parse_args()

    # Find all matching chapter files
    chapter_files = glob.glob(args.chapters, recursive=True)
    chapter_files = [f for f in chapter_files if os.path.isfile(f)]

    if not chapter_files:
        print(f"Error: No chapter files found matching pattern: {args.chapters}")
        sys.exit(1)

    print(f"Found {len(chapter_files)} chapter files")

    # Run ingestion
    ingest_chapters(chapter_files, args.collection)
