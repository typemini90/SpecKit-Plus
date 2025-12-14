import React, { useState, useRef, useEffect } from 'react';

// SSR-safe API_BASE definition
const getApiBase = () => {
  if (typeof window !== 'undefined' && window.location) {
    return window.location.hostname === "localhost"
      ? "http://localhost:8000"
      : "https://speckit-plus-production.up.railway.app";
  }
  // Default to production URL during SSR
  return "https://speckit-plus-production.up.railway.app";
};

const API_BASE = getApiBase();

// --- Icons ---
const Icons = {
  Send: () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><line x1="22" y1="2" x2="11" y2="13"></line><polygon points="22 2 15 22 11 13 2 9 22 2"></polygon></svg>
  ),
  // Changed from Paperclip to Quote/Text icon to avoid "Upload" confusion
  Quote: () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M3 21c3 0 7-1 7-8V5c0-1.25-.756-2.017-2-2H4c-1.25 0-2 .75-2 1.972V11c0 1.25.75 2 2 2 1 0 1 0 1 1v1c0 1-1 2-2 2s-1 .008-1 1.031V20c0 1 0 1 1 1z"></path><path d="M15 21c3 0 7-1 7-8V5c0-1.25-.756-2.017-2-2h-4c-1.25 0-2 .75-2 1.972V11c0 1.25.75 2 2 2 1 0 1 0 1 1v1c0 1-1 2-2 2s-1 .008-1 1.031V20c0 1 0 1 1 1z"></path></svg>
  ),
  Close: () => (
    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
  ),
  Book: () => (
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"></path><path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"></path></svg>
  ),
  ChevronDown: () => (
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><polyline points="6 9 12 15 18 9"></polyline></svg>
  ),
  ChevronUp: () => (
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><polyline points="18 15 12 9 6 15"></polyline></svg>
  )
};

// --- Theme Config ---
const theme = {
  bg: '#050a07',
  primary: '#00ff41',
  primaryDim: 'rgba(0, 255, 65, 0.4)',
  bubbleUser: 'rgba(0, 255, 65, 0.15)',
  bubbleBot: 'transparent',
  border: '#1a3320',
  text: '#ccffda'
};

// --- Sub-component for Source Accordion ---
const SourceDropdown = ({ sources }) => {
  const [isOpen, setIsOpen] = useState(false);

  if (!sources || sources.length === 0) return null;

  return (
    <div style={{ marginTop: '8px', borderTop: `1px solid ${theme.border}` }}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        style={{
          display: 'flex', alignItems: 'center', gap: '6px',
          background: 'none', border: 'none',
          color: theme.primaryDim, fontSize: '11px',
          padding: '8px 0', cursor: 'pointer',
          width: '100%', textAlign: 'left',
          fontFamily: 'inherit'
        }}
      >
        {isOpen ? <Icons.ChevronUp /> : <Icons.ChevronDown />}
        <span>{isOpen ? 'Hide References' : `Show References (${sources.length})`}</span>
      </button>

      {isOpen && (
        <div style={{ display: 'flex', flexWrap: 'wrap', gap: '6px', paddingBottom: '5px', animation: 'fadeIn 0.3s' }}>
          {sources.map((src, idx) => {
             const cleanName = src.replace('../docs/', '').replace('../', '').replace('.pdf', '').replace(/_/g, ' ');
             return (
              <span key={idx} style={{
                display: 'flex', alignItems: 'center', gap: '4px',
                fontSize: '11px', padding: '4px 8px',
                borderRadius: '12px', backgroundColor: 'rgba(0, 255, 65, 0.05)',
                border: `1px solid ${theme.border}`, color: theme.primary
              }}>
                <Icons.Book /> {cleanName}
              </span>
             );
          })}
        </div>
      )}
    </div>
  );
};

const ChatKitInterface = ({ conversationId = 'default-conversation', isEmbedded = false }) => {
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: `Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.`,
      sender: 'bot',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      sources: []
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [stagedSelection, setStagedSelection] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => { scrollToBottom(); }, [messages, isLoading, stagedSelection]);

  const getSelectedText = () => {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    return selection ? selection.toString().trim() : '';
  };

  const handleCaptureSelection = () => {
    const selected = getSelectedText();
    if (!selected) {
        alert("Please highlight text on the page first, then click this Quote button.");
        return;
    }
    if (selected.length > 2000) return alert('Selection too long. Please pick a shorter section.');

    setStagedSelection(selected);
    if (window.getSelection) window.getSelection().removeAllRanges();
  };

  const clearStagedSelection = () => {
    setStagedSelection(null);
  };

  const handleSend = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const actualQuestion = inputValue.trim();
    const contextToSend = stagedSelection;

    const userMessage = {
      id: Date.now(),
      text: actualQuestion,
      context: contextToSend,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setStagedSelection(null);
    setIsLoading(true);

    try {
      const response = await fetch(
        contextToSend
          ? `${API_BASE}/api/selection`
          : `${API_BASE}/api/query`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(
            contextToSend
              ? { selected_text: contextToSend, question: actualQuestion }
              : { query: actualQuestion }
          ),
        }
      );

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: contextToSend ? [] : data.sources || [],
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, botMessage]);

    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        id: Date.now() + 1,
        text: 'System Error: Unable to connect to backend neural network.',
        sender: 'bot',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      }]);
    } finally { setIsLoading(false); }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); handleSend(e); }
  };

  return (
    <div style={{
      display: 'flex', flexDirection: 'column',
      height: isEmbedded ? '400px' : '650px',
      width: isEmbedded ? '100%' : '100%',
      maxWidth: isEmbedded ? '100%' : '800px',
      margin: isEmbedded ? '0' : '20px auto',
      backgroundColor: theme.bg,
      border: `1px solid ${theme.border}`,
      boxShadow: '0 10px 30px rgba(0,0,0,0.5), 0 0 20px rgba(0, 255, 65, 0.05)',
      fontFamily: '"Courier New", Courier, monospace',
      overflow: 'hidden',
      position: 'relative'
    }}>

      {/* Messages Area */}
      <div style={{
        flex: 1,
        padding: '20px',
        overflowY: 'auto',
        display: 'flex',
        flexDirection: 'column',
        gap: '20px',
        scrollBehavior: 'smooth'
      }}>
        {messages.map(msg => (
          <div key={msg.id} style={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: msg.sender === 'user' ? 'flex-end' : 'flex-start',
            maxWidth: '100%'
          }}>

            {/* Context Bubble (If user selected text) */}
            {msg.context && (
              <div style={{
                maxWidth: '85%',
                marginBottom: '4px',
                padding: '8px 12px',
                borderRadius: '8px',
                borderLeft: `3px solid ${theme.primary}`,
                backgroundColor: 'rgba(255,255,255,0.03)',
                fontSize: '11px',
                color: theme.primaryDim,
                fontStyle: 'italic',
                whiteSpace: 'nowrap',
                overflow: 'hidden',
                textOverflow: 'ellipsis',
                alignSelf: 'flex-end'
              }}>
                Reference: "{msg.context.substring(0, 60)}..."
              </div>
            )}

            {/* Main Message Bubble */}
            <div style={{
              maxWidth: '85%',
              padding: '14px 18px',
              borderRadius: msg.sender === 'user' ? '16px 16px 4px 16px' : '16px 16px 16px 4px',
              backgroundColor: msg.sender === 'user' ? theme.bubbleUser : theme.bubbleBot,
              border: msg.sender === 'user' ? 'none' : `1px solid ${theme.border}`,
              color: theme.text,
              lineHeight: '1.6',
              fontSize: '14px',
              boxShadow: msg.sender === 'user' ? 'none' : '0 4px 10px rgba(0,0,0,0.2)'
            }}>
              {msg.text}

              {/* Dropdown References */}
              {msg.sources && msg.sources.length > 0 && (
                <SourceDropdown sources={msg.sources} />
              )}
            </div>

            {/* Timestamp */}
            <div style={{ fontSize: '10px', opacity: 0.4, marginTop: '5px', color: theme.primary, marginLeft: '5px' }}>
              {msg.sender === 'bot' ? 'AI' : 'You'} • {msg.timestamp}
            </div>
          </div>
        ))}

        {/* Thinking Indicator */}
        {isLoading && (
          <div style={{ display: 'flex', flexDirection: 'column', gap: '5px', marginLeft: '10px' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px', color: theme.primary }}>
               <span className="blink-caret" style={{ fontSize: '14px' }}>&gt; System thinking...</span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div style={{
        padding: '15px 20px 20px 20px',
        borderTop: `1px solid ${theme.border}`,
        backgroundColor: 'rgba(0, 20, 10, 0.4)',
        position: 'relative'
      }}>

        {/* Selection Preview Badge */}
        {stagedSelection && (
          <div style={{
            display: 'flex', alignItems: 'center', justifyContent: 'space-between',
            padding: '8px 12px', marginBottom: '10px',
            backgroundColor: 'rgba(0, 255, 65, 0.1)',
            border: `1px solid ${theme.primaryDim}`,
            borderRadius: '8px',
            fontSize: '12px', color: theme.text
          }}>
            <div style={{ display: 'flex', gap: '8px', overflow: 'hidden' }}>
              <span style={{ color: theme.primary, fontWeight: 'bold' }}>CONTEXT:</span>
              <span style={{ fontStyle: 'italic', opacity: 0.8, whiteSpace: 'nowrap' }}>
                "{stagedSelection.substring(0, 50)}..."
              </span>
            </div>
            <button onClick={clearStagedSelection} style={{
              background: 'none', border: 'none', color: theme.primary, cursor: 'pointer', padding: '4px'
            }}>
              <Icons.Close />
            </button>
          </div>
        )}

        {/* Input Form */}
        <form onSubmit={handleSend} style={{ display: 'flex', gap: '10px', alignItems: 'flex-end' }}>

          {/* QUOTE / SELECTION BUTTON */}
          {!stagedSelection && (
            <button
              type="button"
              onClick={handleCaptureSelection}
              title="Highlight text then click here to quote it"
              style={{
                height: '46px', width: '46px',
                display: 'flex', alignItems: 'center', justifyContent: 'center',
                borderRadius: '12px',
                border: `1px solid ${theme.border}`,
                backgroundColor: 'rgba(0,0,0,0.3)',
                color: theme.primary,
                cursor: 'pointer',
                transition: 'all 0.2s'
              }}
              onMouseEnter={e => e.currentTarget.style.backgroundColor = 'rgba(0,255,65,0.1)'}
              onMouseLeave={e => e.currentTarget.style.backgroundColor = 'rgba(0,0,0,0.3)'}
            >
              <Icons.Quote />
            </button>
          )}

          <div style={{ flex: 1, position: 'relative' }}>
            <input
              type="text"
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              disabled={isLoading}
              placeholder={stagedSelection ? "Ask about this quote..." : "Ask about robotics, AI..."}
              style={{
                width: '100%',
                padding: '14px 16px',
                border: `1px solid ${theme.border}`,
                borderRadius: '12px',
                backgroundColor: 'rgba(0,0,0,0.3)',
                color: theme.text,
                outline: 'none',
                fontFamily: 'monospace',
                fontSize: '14px'
              }}
              onFocus={e => e.target.style.borderColor = theme.primary}
              onBlur={e => e.target.style.borderColor = theme.border}
            />
          </div>

          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading}
            style={{
              height: '46px', width: '46px',
              border: 'none',
              borderRadius: '12px',
              backgroundColor: (!inputValue.trim() || isLoading) ? '#1a3320' : theme.primary,
              color: (!inputValue.trim() || isLoading) ? '#446650' : '#001a0d',
              cursor: (!inputValue.trim() || isLoading) ? 'default' : 'pointer',
              display: 'flex', alignItems: 'center', justifyContent: 'center',
              transition: 'background-color 0.2s'
            }}>
            <Icons.Send />
          </button>
        </form>
      </div>

      <style jsx>{`
        .blink-caret { animation: blink 1s step-end infinite; }
        @keyframes blink { 50% { opacity: 0; } }
        @keyframes fadeIn { from { opacity: 0; transform: translateY(-5px); } to { opacity: 1; transform: translateY(0); } }
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: rgba(0, 255, 65, 0.2); border-radius: 3px; }
        ::-webkit-scrollbar-thumb:hover { background: rgba(0, 255, 65, 0.4); }
      `}</style>
    </div>
  );
};

export default ChatKitInterface;
