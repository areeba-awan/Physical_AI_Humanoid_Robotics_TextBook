# Quickstart: RAG Chatbot Integration

## Prerequisites

- Python 3.11+ with pip
- Node.js 18+ with npm
- Access to OpenRouter API
- Access to Qwen embeddings API
- Qdrant Cloud account
- Neon Postgres account

## Environment Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Create a `.env` file in the backend directory with the following variables:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
NEON_DATABASE_URL=your_neon_postgres_connection_string
QWEN_EMBEDDINGS_MODEL=qwen-embedding-model-name
OPENROUTER_MODEL=openchat/openchat-7b
QDRANT_COLLECTION_NAME=book_content
EMBEDDING_DIMENSION=768
MAX_CHUNK_SIZE=1000
RETRIEVAL_LIMIT=5
SESSION_TIMEOUT_MINUTES=30
MAX_CONTEXT_CHUNKS=5
```

## Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Run the backend server:
```bash
uvicorn src.main:app --reload --port 8000
```

## Frontend Setup

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm run dev
```

## Running Tests

### Backend Tests
```bash
cd backend
python -m pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm run test
```

## Key Components

### Book Ingestion
- Run the ingestion pipeline: `python -m src.scripts.ingest_book --book-path path/to/book.pdf`
- This processes the book, chunks content, generates embeddings, and stores in Qdrant

### RAG Service
- Located in `src/services/rag_service.py`
- Handles the retrieval and generation workflow
- Manages context filtering for selected-text queries

### API Endpoints
- Query endpoint: `POST /v1/book/{book_id}/query`
- Selected text query: `POST /v1/book/{book_id}/query-selected`
- Session management: `POST /v1/session`

## Configuration

The application can be configured through environment variables in the `.env` file:

- `OPENROUTER_MODEL`: The model to use for generation (default: "openchat/openchat-7b")
- `QDRANT_COLLECTION_NAME`: The collection to use for vector storage (default: "book_content")
- `EMBEDDING_DIMENSION`: Dimension of embeddings (default: 768)
- `MAX_CHUNK_SIZE`: Maximum size of text chunks (default: 1000 characters)
- `RETRIEVAL_LIMIT`: Number of chunks to retrieve (default: 5)
- `SESSION_TIMEOUT_MINUTES`: Minutes until session expires (default: 30)

## Adding a New Book

1. Prepare your book in a text-compatible format (PDF, EPUB, or plain text)
2. Run the ingestion script:
```bash
python -m src.scripts.ingest_book --book-path path/to/book.pdf --book-id unique-book-identifier
```
3. The script will:
   - Parse the book content
   - Split content into semantic chunks
   - Generate embeddings using Qwen
   - Store chunks in Qdrant with metadata
   - Create book metadata in Neon Postgres

## Troubleshooting

- If you see embedding errors, ensure your Qwen API keys are correctly configured
- If vector search returns no results, check that your book was correctly ingested
- For LLM response errors, verify your OpenRouter configuration

## Development Workflow

1. Make changes to the code
2. Run relevant unit tests: `python -m pytest tests/unit/`
3. Run integration tests: `python -m pytest tests/integration/`
4. Verify API contracts still pass: `python -m pytest tests/contract/`
5. Test end-to-end functionality with manual testing or e2e tests

## Deployment

### Backend
1. Build the backend application:
```bash
python -m build
```

2. Deploy to your preferred hosting platform (AWS, Azure, GCP, etc.)

3. Ensure environment variables are properly configured in the deployment environment

### Frontend
1. Build the frontend application:
```bash
cd frontend && npm run build
```

2. Serve the build output via a web server (nginx, Apache, etc.)

## Performance Optimization

1. Adjust the embedding model based on your needs (faster but less accurate vs. slower but more accurate)
2. Tune the chunk size based on your content and retrieval needs
3. Configure caching layers appropriately for your usage patterns
4. Monitor API costs and adjust model usage accordingly

## Security Best Practices

1. Never commit API keys or sensitive credentials to version control
2. Use environment variables for all sensitive configuration
3. Implement proper rate limiting to prevent abuse
4. Sanitize all user inputs to prevent injection attacks
5. Use HTTPS in production environments

## Monitoring and Logging

1. The application logs important events and errors using Python's logging module
2. Monitor for performance metrics like response times and error rates
3. Set up alerts for critical failures or unusual activity patterns
4. Log user interactions to understand usage patterns (while respecting privacy)