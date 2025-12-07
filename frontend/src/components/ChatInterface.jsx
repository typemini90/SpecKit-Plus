import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const ChatInterface = ({ selectedText = null }) => {
  const { siteConfig } = useDocusaurusContext();
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: `Hello! I'm your AI assistant for the Embodied Intelligence textbook. Ask me anything about robotics, AI, or the content!`,
      sender: 'bot',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      sources: []
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSelectedText, setCurrentSelectedText] = useState(selectedText);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    return selection ? selection.toString().trim() : '';
  };

  // Function to handle asking about selected text
  const handleAskAboutSelection = () => {
    const selected = getSelectedText() || currentSelectedText;
    if (!selected) {
      alert('Please select some text first, or provide text via the selectedText prop.');
      return;
    }

    if (selected.length > 2000) { // Limit text length
      alert('Selected text is too long. Please select a shorter portion.');
      return;
    }

    const questionPrompt = `About this text: "${selected}". Question: `;
    setInputValue(questionPrompt);
    setCurrentSelectedText(selected);
  };

  const handleSend = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Check if this is a selection-based query
    const isSelectionQuery = currentSelectedText && inputValue.startsWith(`About this text: "${currentSelectedText}" Question: `);
    const actualQuestion = isSelectionQuery
      ? inputValue.replace(`About this text: "${currentSelectedText}" Question: `, '').trim()
      : inputValue;

    const userMessage = {
      id: Date.now(),
      text: inputValue, // Store the full input as the user sees it
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      let response;
      let data;

      if (isSelectionQuery && currentSelectedText) {
        // Use selection endpoint
        response = await fetch('/api/selection', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            selected_text: currentSelectedText,
            question: actualQuestion
          }),
        });
      } else {
        // Use regular query endpoint
        response = await fetch('/api/query', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ query: inputValue }),
        });
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: isSelectionQuery ? [] : data.sources || [], // Selection responses don't have sources
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, botMessage]);
      setInputValue(''); // Clear input after successful send
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please make sure the backend server is running.',
        sender: 'bot',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };
      setMessages(prev => [...prev, errorMessage]);
      setInputValue(''); // Clear input on error too
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend(e);
    }
  };

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '600px',
      border: '1px solid #00ff41',
      borderRadius: '8px',
      backgroundColor: '#001a0d',
      color: '#00ff41',
      fontFamily: 'monospace',
      overflow: 'hidden',
      maxWidth: '800px',
      margin: '20px 0'
    }}>
      {/* Chat Header */}
      <div style={{
        padding: '15px',
        backgroundColor: 'rgba(0, 255, 65, 0.1)',
        borderBottom: '1px solid #00ff41',
        display: 'flex',
        alignItems: 'center',
        gap: '10px'
      }}>
        <div style={{
          width: '12px',
          height: '12px',
          borderRadius: '50%',
          backgroundColor: '#00ff41',
          boxShadow: '0 0 8px #00ff41'
        }}></div>
        <span style={{ fontWeight: 'bold' }}>AI Assistant</span>
        <span style={{ fontSize: '12px', opacity: 0.7 }}>
          Ask about Embodied Intelligence & Robotics
        </span>
      </div>

      {/* Messages Container */}
      <div style={{
        flex: 1,
        padding: '20px',
        overflowY: 'auto',
        display: 'flex',
        flexDirection: 'column',
        gap: '15px'
      }}>
        {messages.map((message) => (
          <div
            key={message.id}
            style={{
              display: 'flex',
              justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start',
              animation: 'fadeIn 0.3s ease-in'
            }}
          >
            <div style={{
              maxWidth: '80%',
              padding: '12px 16px',
              borderRadius: '18px',
              backgroundColor: message.sender === 'user'
                ? 'rgba(0, 204, 68, 0.2)'
                : 'rgba(0, 255, 65, 0.1)',
              border: message.sender === 'user'
                ? '1px solid rgba(0, 204, 68, 0.3)'
                : '1px solid rgba(0, 255, 65, 0.2)',
              color: message.sender === 'user' ? '#00ff41' : '#00ff41',
              wordWrap: 'break-word',
              lineHeight: '1.5'
            }}>
              <div style={{ marginBottom: '5px', fontSize: '14px' }}>
                {message.text}
              </div>
              {message.sources && message.sources.length > 0 && (
                <div style={{
                  fontSize: '11px',
                  opacity: 0.7,
                  marginTop: '5px',
                  paddingTop: '5px',
                  borderTop: '1px solid rgba(0, 255, 65, 0.2)'
                }}>
                  Sources: {message.sources.map((src, idx) => (
                    <span key={idx} style={{ display: 'block', marginTop: '2px' }}>
                      {src.replace('../docs/', '').replace('../', '')}
                    </span>
                  ))}
                </div>
              )}
              <div style={{
                fontSize: '10px',
                opacity: 0.6,
                textAlign: 'right',
                marginTop: '5px'
              }}>
                {message.timestamp}
              </div>
            </div>
          </div>
        ))}

        {isLoading && (
          <div style={{
            display: 'flex',
            justifyContent: 'flex-start',
            padding: '0 20px 20px 20px'
          }}>
            <div style={{
              padding: '12px 16px',
              borderRadius: '18px',
              backgroundColor: 'rgba(0, 255, 65, 0.1)',
              border: '1px solid rgba(0, 255, 65, 0.2)',
              color: '#00ff41'
            }}>
              <div style={{ display: 'flex', gap: '4px' }}>
                <div style={{
                  width: '6px',
                  height: '6px',
                  borderRadius: '50%',
                  backgroundColor: '#00ff41',
                  animation: 'pulse 1.5s infinite'
                }}></div>
                <div style={{
                  width: '6px',
                  height: '6px',
                  borderRadius: '50%',
                  backgroundColor: '#00ff41',
                  animation: 'pulse 1.5s infinite',
                  animationDelay: '0.2s'
                }}></div>
                <div style={{
                  width: '6px',
                  height: '6px',
                  borderRadius: '50%',
                  backgroundColor: '#00ff41',
                  animation: 'pulse 1.5s infinite',
                  animationDelay: '0.4s'
                }}></div>
              </div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div style={{
        padding: '15px',
        borderTop: '1px solid #00ff41',
        backgroundColor: 'rgba(0, 255, 65, 0.05)'
      }}>
        <form onSubmit={handleSend} style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <div style={{ display: 'flex', gap: '10px' }}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics, AI, or textbook content..."
              disabled={isLoading}
              style={{
                flex: 1,
                padding: '12px 16px',
                border: '1px solid #00ff41',
                borderRadius: '24px',
                backgroundColor: '#001a0d',
                color: '#00ff41',
                outline: 'none',
                fontFamily: 'monospace'
              }}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              style={{
                padding: '12px 24px',
                border: 'none',
                borderRadius: '24px',
                backgroundColor: '#00cc44',
                color: 'white',
                cursor: (!inputValue.trim() || isLoading) ? 'not-allowed' : 'pointer',
                opacity: (!inputValue.trim() || isLoading) ? 0.5 : 1,
                transition: 'all 0.2s ease',
                fontFamily: 'monospace',
                fontWeight: 'bold'
              }}
              onMouseEnter={(e) => {
                if (!(!inputValue.trim() || isLoading)) {
                  e.target.style.backgroundColor = '#00ff41';
                  e.target.style.textShadow = '0 0 8px rgba(0, 255, 65, 0.6)';
                }
              }}
              onMouseLeave={(e) => {
                if (!(!inputValue.trim() || isLoading)) {
                  e.target.style.backgroundColor = '#00cc44';
                  e.target.style.textShadow = 'none';
                }
              }}
            >
              {isLoading ? '...' : 'Send'}
            </button>
          </div>
          <button
            type="button"
            onClick={handleAskAboutSelection}
            disabled={isLoading}
            style={{
              padding: '8px 12px',
              border: '1px solid #00ff41',
              borderRadius: '24px',
              backgroundColor: 'transparent',
              color: '#00ff41',
              cursor: isLoading ? 'not-allowed' : 'pointer',
              opacity: isLoading ? 0.5 : 1,
              transition: 'all 0.2s ease',
              fontFamily: 'monospace',
              fontSize: '14px'
            }}
            onMouseEnter={(e) => {
              if (!isLoading) {
                e.target.style.backgroundColor = 'rgba(0, 255, 65, 0.1)';
              }
            }}
            onMouseLeave={(e) => {
              if (!isLoading) {
                e.target.style.backgroundColor = 'transparent';
              }
            }}
          >
            Ask about selected text
          </button>
        </form>
        <div style={{
          fontSize: '11px',
          opacity: 0.6,
          textAlign: 'center',
          marginTop: '8px'
        }}>
          Powered by Gemini AI and textbook content
        </div>
      </div>

      <style jsx>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }

        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }
      `}</style>
    </div>
  );
};

export default ChatInterface;