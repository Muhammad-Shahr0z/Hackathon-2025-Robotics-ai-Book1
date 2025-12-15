import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
import time

# -------------------------------------
# CONFIG
# -------------------------------------
SITEMAP_URL = "https://muhammad-shahr0z.github.io/Hackathon-2025-Robotics-ai-Book1/sitemap.xml"
COLLECTION_NAME = "AiBook_COllection-01"

cohere_client = cohere.Client("QCG5iT2kxP6WkYQ8qNYxY35jQ34dOjjsdG7vVuVp")
EMBED_MODEL = "embed-english-v3.0"

qdrant = QdrantClient(
   api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ZbAKFJwRpuUl-jRzVSb1tmpo7otgELHPzKvcFoMRjg0",
   url="https://8dadf528-35a4-4732-a9de-8ca0c7284c64.us-east4-0.gcp.cloud.qdrant.io"
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml_data = requests.get(sitemap_url, timeout=20).text
    root = ET.fromstring(xml_data)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls

# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    try:
        html = requests.get(url, timeout=20).text
        text = trafilatura.extract(html)
        if not text:
            print("[WARNING] No text extracted from:", url)
        return text
    except Exception as e:
        print(f"[ERROR] Failed to fetch {url}: {e}")
        return None

# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    if text.strip():
        chunks.append(text)
    return chunks

# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_document",
        texts=[text],
    )
    return response.embeddings[0]

# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,
            distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)
    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )
    time.sleep(0.1)  # optional: avoid hitting rate limits

# -------------------------------------
# Helper — Only valid content URLs
# -------------------------------------
def is_valid_content_url(url):
    return (
        "/chapter-" in url
        or url.endswith("/capstone")
        or url.endswith("/markdown-page")
    )

# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)
    create_collection()

    global_id = 1

    for url in urls:
        if not is_valid_content_url(url):
            print("Skipping index/irrelevant page:", url)
            continue

        print("\nProcessing:", url)
        text = extract_text_from_url(url)
        if not text:
            continue

        chunks = chunk_text(text)
        for i, ch in enumerate(chunks):
            print(f"Embedding chunk {i+1}/{len(chunks)}")
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

    print("\n✔️ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)

if __name__ == "__main__":
    ingest_book()
