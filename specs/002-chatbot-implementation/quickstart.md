# Quickstart: Chatbot Implementation

## Overview
This guide provides instructions for implementing and using the chatbot interface for the Embodied Intelligence textbook.

## Prerequisites
- Backend RAG system must be running and properly indexed
- Environment variables must be configured (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- Frontend development server must be running

## Frontend Component Implementation

### 1. Create the Chat Component
```bash
# The component will be created at:
frontend/src/components/ChatInterface.jsx
```

### 2. Component Features
- Real-time chat interface with user and bot messages
- Loading indicators during AI processing
- Source document references in bot responses
- Error handling for API failures
- Matrix-themed design consistent with the rest of the site

### 3. Integration Points
- Uses existing `/api/query` endpoint for general questions
- Will integrate with `/api/selection` for text selection-based questions
- Follows existing error handling patterns

## Backend API Endpoints (Already Implemented)

### Query Endpoint
- **URL**: `/api/query`
- **Method**: POST
- **Request**: `{ "query": "your question here" }`
- **Response**: `{ "answer": "AI response", "sources": ["doc paths"] }`

### Selection Endpoint
- **URL**: `/api/selection`
- **Method**: POST
- **Request**: `{ "selected_text": "highlighted text", "question": "your question" }`
- **Response**: `{ "answer": "AI response" }`

## Running the Chatbot

### Development
1. Start the backend server:
```bash
cd backend
uvicorn main:app --reload --port 8000
```

2. Start the frontend development server:
```bash
cd frontend
npm run start
```

3. The chatbot will be available as a component that can be embedded in any page

### Testing
- Test with various questions about the textbook content
- Verify source document references are included
- Test error handling when backend is unavailable
- Verify responsive design on different screen sizes

## Usage in Docusaurus Pages

The chatbot component can be imported and used in any MDX file:

```mdx
import ChatInterface from '@site/src/components/ChatInterface';

<ChatInterface />
```