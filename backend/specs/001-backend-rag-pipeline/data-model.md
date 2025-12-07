# Data Model: Backend RAG Pipeline

## Entities

### DocumentChunk
Represents a segment of text extracted from .md/.mdx files, with associated embedding vector

**Fields**:
- `chunk_id` (str): Unique identifier for the chunk (primary key)
- `content` (str): The actual text content of the chunk
- `doc_path` (str): Path to the original document file
- `embedding` (List[float]): 384-dimensional vector embedding of the content
- `metadata` (Dict[str, Any]): Additional document metadata (title, section, etc.)

**Validation Rules**:
- `chunk_id` must be unique
- `content` must not be empty
- `embedding` must be exactly 384 dimensions
- `doc_path` must exist and point to a valid file

### Query
A question submitted by the user for contextual answer generation

**Fields**:
- `query` (str): The user's question text

**Validation Rules**:
- `query` must not be empty
- `query` must be less than 1000 characters

### Selection
User-selected text content provided for targeted answer generation

**Fields**:
- `selected_text` (str): The text selected by the user
- `question` (str): The question about the selected text

**Validation Rules**:
- `selected_text` must not be empty
- `selected_text` must be less than 10,000 characters
- `question` must not be empty
- `question` must be less than 1000 characters

### Response
Structured answer from the system with appropriate status information

**Fields**:
- `answer` (str): The generated answer text
- `sources` (List[str]): List of source documents that informed the answer (for query responses)

**Validation Rules**:
- `answer` must not be empty unless system is explicitly stating "I don't know"
- `sources` is optional for selection responses

## Relationships

### DocumentChunk → Document
- Each `DocumentChunk` originates from a single document file (represented by `doc_path`)
- One document can generate multiple chunks

## State Transitions

### Document Indexing Process
1. **Document Read**: Raw .md/.mdx file is read
2. **Document Chunked**: File is split into `DocumentChunk` entities
3. **Embedding Generated**: Each chunk gets a 384-dimensional embedding
4. **Stored in Qdrant**: Chunk and embedding are stored in vector database

### Query Process
1. **Query Received**: User submits a `Query`
2. **Similarity Search**: Qdrant is searched for relevant `DocumentChunk` entities
3. **Context Collected**: Retrieved chunks form the context for answer generation
4. **Answer Generated**: Using Gemini API with context
5. **Response Formed**: `Response` is created and returned to user

### Selection Process
1. **Selection Received**: User submits `Selection` with text and question
2. **Context Formed**: Selected text is used as context for answer generation
3. **Answer Generated**: Using Gemini API with provided context
4. **Response Formed**: `Response` is created and returned to user