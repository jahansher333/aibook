#!/usr/bin/env python3
"""
Ingestion script - ingests ALL chapters from Physical AI Textbook into Qdrant
Processes all chapter markdown files (index.md and ch*.md) from docusaurus/docs/
"""

import os
import sys
import re
import glob
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
    """Parse chapter file and extract content with metadata"""
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

        # Remove import statements (React components)
        content_clean = re.sub(r'^import\s+.*$', '', main_content, flags=re.MULTILINE)

        # Remove JSX component tags
        content_clean = re.sub(r'<[A-Z][a-zA-Z]*[^>]*/>', '', content_clean)
        content_clean = re.sub(r'<[A-Z][a-zA-Z]*[^>]*>.*?</[A-Z][a-zA-Z]*>', '', content_clean, flags=re.DOTALL)

        # Remove code blocks (keep the text around them)
        content_clean = re.sub(r'```[\s\S]*?```', '', content_clean)

        # Remove Mermaid diagrams
        content_clean = re.sub(r'```mermaid[\s\S]*?```', '', content_clean)

        # Clean up multiple newlines
        content_clean = re.sub(r'\n{3,}', '\n\n', content_clean)

        return {
            'chapter_id': chapter_id,
            'chapter_title': chapter_title,
            'content': content_clean.strip(),
            'file_path': str(file_path)
        }

    return None

def chunk_text(text, max_tokens=512, overlap=50):
    """Chunk text into overlapping segments"""
    tokens = encoding.encode(text)
    chunks = []

    for i in range(0, len(tokens), max_tokens - overlap):
        chunk_tokens = tokens[i:i + max_tokens]
        chunk_text = encoding.decode(chunk_tokens)
        if chunk_text.strip():  # Only add non-empty chunks
            chunks.append(chunk_text)

    return chunks

def find_all_chapters(docs_path):
    """Find all chapter markdown files"""
    chapters = []

    # Get the absolute path to docs directory
    script_dir = Path(__file__).parent
    docs_dir = script_dir.parent.parent / "docusaurus" / "docs"

    print(f"Looking for chapters in: {docs_dir}")

    # Find all chapter directories (ch01*, ch02*, etc.)
    chapter_dirs = sorted(docs_dir.glob("ch*"))

    for chapter_dir in chapter_dirs:
        if chapter_dir.is_dir():
            # Look for index.md first (main chapter content)
            index_file = chapter_dir / "index.md"
            if index_file.exists():
                chapters.append(index_file)
            else:
                # Fallback to ch*.md files
                ch_files = list(chapter_dir.glob("ch*.md"))
                chapters.extend(ch_files)

    # Also include intro.md and hardware-requirements.md
    intro_file = docs_dir / "intro.md"
    if intro_file.exists():
        chapters.insert(0, intro_file)

    hardware_file = docs_dir / "hardware-requirements.md"
    if hardware_file.exists():
        chapters.append(hardware_file)

    return chapters

def main():
    collection_name = os.getenv("QDRANT_COLLECTION", "physical_ai_book")

    print("=" * 60)
    print("Physical AI Textbook - Qdrant Ingestion Script")
    print("=" * 60)

    # Find all chapter files
    chapter_files = find_all_chapters("docusaurus/docs")

    if not chapter_files:
        print("ERROR: No chapter files found!")
        return

    print(f"\nFound {len(chapter_files)} chapter files to ingest:")
    for f in chapter_files:
        print(f"  - {f.name} ({f.parent.name})")

    all_points = []
    point_id = 0
    total_chunks = 0

    for idx, chapter_file in enumerate(chapter_files):
        print(f"\n[{idx + 1}/{len(chapter_files)}] Processing: {chapter_file.parent.name}/{chapter_file.name}")

        # Parse chapter
        chapter = parse_chapter(chapter_file)
        if not chapter:
            print(f"  WARNING: Could not parse {chapter_file}, skipping...")
            continue

        print(f"  Chapter ID: {chapter['chapter_id']}")
        print(f"  Title: {chapter['chapter_title']}")

        # Check if content is too short
        if len(chapter['content']) < 100:
            print(f"  WARNING: Content too short ({len(chapter['content'])} chars), skipping...")
            continue

        # Chunk content
        chunks = chunk_text(chapter['content'])
        print(f"  Created {len(chunks)} chunks")

        if not chunks:
            print(f"  WARNING: No chunks created, skipping...")
            continue

        # Generate embeddings (batch for efficiency)
        print(f"  Generating embeddings...")
        try:
            response = co.embed(
                texts=chunks,
                model="embed-english-v3.0",
                input_type="search_document",
                embedding_types=["float"]
            )
            embeddings = response.embeddings.float
        except Exception as e:
            print(f"  ERROR generating embeddings: {e}")
            continue

        # Create points
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            all_points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": chunk,
                    "chapter_id": chapter['chapter_id'],
                    "chapter_title": chapter['chapter_title'],
                    "chunk_index": i,
                    "source_file": chapter_file.name
                }
            ))
            point_id += 1

        total_chunks += len(chunks)
        print(f"  Added {len(chunks)} points (total: {point_id})")

    if not all_points:
        print("\nERROR: No points to upload!")
        return

    # Upload all points to Qdrant in batches
    print(f"\n" + "=" * 60)
    print(f"Uploading {len(all_points)} points to Qdrant collection '{collection_name}'...")

    batch_size = 100
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i + batch_size]
        try:
            qdrant.upsert(
                collection_name=collection_name,
                points=batch
            )
            print(f"  Uploaded batch {i // batch_size + 1}/{(len(all_points) - 1) // batch_size + 1} ({len(batch)} points)")
        except Exception as e:
            print(f"  ERROR uploading batch: {e}")
            return

    print(f"\n" + "=" * 60)
    print("[SUCCESS] Ingestion complete!")
    print(f"  Total chapters processed: {len(chapter_files)}")
    print(f"  Total chunks created: {total_chunks}")
    print(f"  Total points in Qdrant: {len(all_points)}")
    print("=" * 60)

if __name__ == "__main__":
    main()
