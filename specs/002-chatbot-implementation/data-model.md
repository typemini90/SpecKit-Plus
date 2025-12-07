# Data Model: Chatbot Interface

## Message Entity

### Structure
```
Message {
  id: string | number
  text: string
  sender: "user" | "bot"
  timestamp: string (ISO format or formatted time)
  sources: string[] (optional, for bot responses from RAG)
}
```

### Fields
- `id`: Unique identifier for each message in the conversation
- `text`: The actual content of the message
- `sender`: Indicates whether the message is from the user or the bot
- `timestamp`: Time when the message was sent/received
- `sources`: Array of document paths that contributed to the bot's response (from RAG system)

## Chat State

### Structure
```
ChatState {
  messages: Message[]
  isLoading: boolean
  error: string | null
}
```

### Fields
- `messages`: Array of all messages in the current conversation
- `isLoading`: Boolean indicating if the bot is processing a response
- `error`: Any error message that occurred during API communication

## API Request/Response Models

### Query Request (to backend)
```
QueryRequest {
  query: string
}
```

### Query Response (from backend)
```
QueryResponse {
  answer: string
  sources: string[]
}
```

### Selection Request (to backend)
```
SelectionRequest {
  selected_text: string
  question: string
}
```

### Selection Response (from backend)
```
SelectionResponse {
  answer: string
}
```

## Component Props

### ChatInterface Component
```
ChatInterfaceProps {
  initialMessages?: Message[]
  placeholder?: string
  title?: string
}
```

## Validation Rules

1. Message text must not be empty or whitespace-only
2. Sender must be either "user" or "bot"
3. Timestamp must be in valid format
4. Sources array should only exist for bot messages
5. API requests must include required fields