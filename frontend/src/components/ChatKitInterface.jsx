import React, { useState, useRef, useEffect } from 'react';

const API_BASE = "http://localhost:8000"; // FastAPI backend

const ChatKitInterface = ({ conversationId = 'default-conversation', isEmbedded = false }) => {
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: `Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. Ask me anything about robotics, AI, or the content!`,
      sender: 'bot',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      sources: []
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSelectedText, setCurrentSelectedText] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => { scrollToBottom(); }, [messages]);

  const getSelectedText = () => {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    return selection ? selection.toString().trim() : '';
  };

  const handleAskAboutSelection = () => {
    const selected = getSelectedText() || currentSelectedText;
    if (!selected) return alert('Please select some text first.');
    if (selected.length > 2000) return alert('Selected text is too long. Select a shorter portion.');
    const questionPrompt = `About this text: "${selected}". Question: `;
    setInputValue(questionPrompt);
    setCurrentSelectedText(selected);
  };

  const handleSend = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const isSelectionQuery = currentSelectedText && inputValue.startsWith(`About this text: "${currentSelectedText}" Question: `);
    const actualQuestion = isSelectionQuery
      ? inputValue.replace(`About this text: "${currentSelectedText}" Question: `, '').trim()
      : inputValue;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await fetch(
        isSelectionQuery && currentSelectedText 
          ? `${API_BASE}/api/selection` 
          : `${API_BASE}/api/query`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(
            isSelectionQuery 
              ? { selected_text: currentSelectedText, question: actualQuestion }
              : { query: inputValue }
          ),
        }
      );

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: isSelectionQuery ? [] : data.sources || [],
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, botMessage]);
      setInputValue('');
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please make sure the backend server is running.',
        sender: 'bot',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      }]);
      setInputValue('');
    } finally { setIsLoading(false); }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); handleSend(e); }
  };

  return (
    <div style={{
      display: 'flex', flexDirection: 'column',
      height: isEmbedded ? '400px' : '600px',
      border: '1px solid #00ff41', borderRadius: '8px',
      backgroundColor: '#001a0d', color: '#00ff41',
      fontFamily: 'monospace', overflow: 'hidden',
      maxWidth: isEmbedded ? '100%' : '800px',
      margin: isEmbedded ? '0' : '20px 0', width: isEmbedded ? '100%' : 'auto'
    }}>

      {/* Messages */}
      <div style={{ flex: 1, padding: '20px', overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: '15px' }}>
        {messages.map(msg => (
          <div key={msg.id} style={{ display: 'flex', justifyContent: msg.sender === 'user' ? 'flex-end' : 'flex-start' }}>
            <div style={{
              maxWidth: '80%', padding: '12px 16px', borderRadius: '18px',
              backgroundColor: msg.sender === 'user' ? 'rgba(0,204,68,0.2)' : 'rgba(0,255,65,0.1)',
              border: msg.sender === 'user' ? '1px solid rgba(0,204,68,0.3)' : '1px solid rgba(0,255,65,0.2)',
              color: '#00ff41', wordWrap: 'break-word', lineHeight: '1.5'
            }}>
              <div style={{ marginBottom: '5px', fontSize: '14px' }}>{msg.text}</div>
              {msg.sources && msg.sources.length > 0 && (
                <div style={{ fontSize: '11px', opacity: 0.7, marginTop: '5px', paddingTop: '5px', borderTop: '1px solid rgba(0,255,65,0.2)' }}>
                  Sources: {msg.sources.map((src, idx) => <div key={idx}>{src.replace('../docs/', '').replace('../', '')}</div>)}
                </div>
              )}
              <div style={{ fontSize: '10px', opacity: 0.6, textAlign: 'right', marginTop: '5px' }}>{msg.timestamp}</div>
            </div>
          </div>
        ))}
        {isLoading && <div style={{ display: 'flex', justifyContent: 'flex-start', padding: '0 20px 20px 20px' }}>
          <div style={{ padding: '12px 16px', borderRadius: '18px', backgroundColor: 'rgba(0,255,65,0.1)', border: '1px solid rgba(0,255,65,0.2)', color: '#00ff41' }}>
            <div style={{ display: 'flex', gap: '4px' }}>
              <div style={{ width: '6px', height: '6px', borderRadius: '50%', backgroundColor: '#00ff41', animation: 'pulse 1.5s infinite' }}></div>
              <div style={{ width: '6px', height: '6px', borderRadius: '50%', backgroundColor: '#00ff41', animation: 'pulse 1.5s infinite', animationDelay: '0.2s' }}></div>
              <div style={{ width: '6px', height: '6px', borderRadius: '50%', backgroundColor: '#00ff41', animation: 'pulse 1.5s infinite', animationDelay: '0.4s' }}></div>
            </div>
          </div>
        </div>}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div style={{ padding: '15px', borderTop: '1px solid #00ff41', backgroundColor: 'rgba(0,255,65,0.05)' }}>
        <form onSubmit={handleSend} style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <div style={{ display: 'flex', gap: '10px' }}>
            <input
              type="text" value={inputValue} onChange={e => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown} disabled={isLoading}
              placeholder="Ask about robotics, AI, or textbook content..."
              style={{ flex: 1, padding: '12px 16px', border: '1px solid #00ff41', borderRadius: '24px', backgroundColor: '#001a0d', color: '#00ff41', outline: 'none', fontFamily: 'monospace' }}
            />
            <button type="submit" disabled={!inputValue.trim() || isLoading}
              style={{ padding: '12px 24px', border: 'none', borderRadius: '24px', backgroundColor: '#00cc44', color: 'white', cursor: 'pointer', fontFamily: 'monospace', fontWeight: 'bold' }}>
              {isLoading ? '...' : 'Send'}
            </button>
          </div>
          <button type="button" onClick={handleAskAboutSelection} disabled={isLoading}
            style={{ padding: '8px 12px', border: '1px solid #00ff41', borderRadius: '24px', backgroundColor: 'transparent', color: '#00ff41', cursor: 'pointer', fontFamily: 'monospace', fontSize: '14px' }}>
            Ask about selected text
          </button>
        </form>
        <div style={{ fontSize: '11px', opacity: 0.6, textAlign: 'center', marginTop: '8px' }}>
          Powered by OpenAI and textbook content
        </div>
      </div>

      <style jsx>{`
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
      `}</style>
    </div>
  );
};

export default ChatKitInterface;
