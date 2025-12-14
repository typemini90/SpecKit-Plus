import React, { useState } from 'react';
import ChatKitInterface from '@site/src/components/ChatKitInterface';

// Icons for the widget controls
const WidgetIcons = {
  // ... (Chat icon remains same)
  Chat: () => (
    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path></svg>
  ),
  Minimize: () => (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><line x1="5" y1="12" x2="19" y2="12"></line></svg>
  ),
  Maximize: () => (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"></rect></svg>
  ),
  Close: () => (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>
  )
};

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false); // Default to open when clicked

  const toggleChat = () => {
    if (!isOpen) {
      setIsOpen(true);
      setIsMinimized(false);
    } else {
      setIsMinimized(!isMinimized);
    }
  };

  const closeChat = () => {
    setIsOpen(false);
    setIsMinimized(false);
  };

  return (
    <div className="floating-chat-widget">
      {isOpen && (
        <div className={`chat-window ${isMinimized ? 'minimized' : 'expanded'}`}>
          
          {/* Widget Header - Styled like a Terminal Title Bar */}
          <div className="chat-header" onClick={() => setIsMinimized(!isMinimized)}>
            <div className="chat-status">
              <span className="status-dot"></span>
              <span className="chat-title">AI_TUTOR_V2.0</span>
            </div>
            <div className="chat-controls">
              <button
                className="control-btn"
                onClick={(e) => { e.stopPropagation(); toggleChat(); }}
                aria-label={isMinimized ? "Expand" : "Minimize"}
              >
                {isMinimized ? <WidgetIcons.Maximize /> : <WidgetIcons.Minimize />}
              </button>
              <button
                className="control-btn close"
                onClick={(e) => { e.stopPropagation(); closeChat(); }}
                aria-label="Close"
              >
                <WidgetIcons.Close />
              </button>
            </div>
          </div>

          {/* Chat Body - Only renders when expanded */}
          {!isMinimized && (
            <div className="chat-body">
              {/* Note: Ensure ChatKitInterface has isEmbedded={true} to fit container */}
              <ChatKitInterface isEmbedded={true} conversationId="floating-chat" />
            </div>
          )}
        </div>
      )}

      {/* Floating Action Button (FAB) */}
      {!isOpen && (
        <button
          className="chat-fab"
          onClick={() => {
            setIsOpen(true);
            setIsMinimized(false);
          }}
          aria-label="Open AI Assistant"
        >
          <WidgetIcons.Chat />
          <span className="fab-pulse"></span>
        </button>
      )}

      <style jsx>{`
        /* --- Themes --- */
        .floating-chat-widget {
          --bg-dark: #050a07;
          --primary: #00ff41;
          --primary-dim: rgba(0, 255, 65, 0.3);
          --border: #1a3320;
          --text: #ccffda;
          
          position: fixed;
          bottom: 30px;
          right: 30px;
          z-index: 9999;
          font-family: "Courier New", Courier, monospace;
        }

        /* --- FAB Button --- */
        .chat-fab {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background: #001a0d;
          color: var(--primary);
          border: 1px solid var(--primary);
          cursor: pointer;
          box-shadow: 0 0 15px var(--primary-dim), 0 5px 20px rgba(0,0,0,0.5);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.3s ease;
          position: relative;
        }

        .chat-fab:hover {
          background: var(--primary);
          color: #000;
          transform: translateY(-2px);
          box-shadow: 0 0 25px var(--primary-dim);
        }

        /* Pulse Animation Ring */
        .fab-pulse {
          position: absolute;
          top: 0; left: 0; right: 0; bottom: 0;
          border-radius: 50%;
          border: 1px solid var(--primary);
          animation: pulse-ring 2s infinite;
          opacity: 0;
        }

        @keyframes pulse-ring {
          0% { transform: scale(1); opacity: 0.5; }
          100% { transform: scale(1.5); opacity: 0; }
        }

        /* --- Chat Window --- */
        .chat-window {
          width: 400px; /* Slightly wider for code readability */
          display: flex;
          flex-direction: column;
          border-radius: 16px;
          overflow: hidden;
          background: var(--bg-dark);
          border: 1px solid var(--primary);
          box-shadow: 0 20px 50px rgba(0,0,0,0.8), 0 0 0 1px rgba(0,255,65,0.1);
          animation: slideUp 0.3s cubic-bezier(0.16, 1, 0.3, 1);
        }

        @keyframes slideUp {
          from { opacity: 0; transform: translateY(20px) scale(0.95); }
          to { opacity: 1; transform: translateY(0) scale(1); }
        }

        /* --- Header --- */
        .chat-header {
          background: rgba(0, 20, 10, 0.95);
          border-bottom: 1px solid var(--border);
          padding: 20px 16px;
          display: flex;
          justify-content: space-between;
          align-items: center;
          cursor: pointer;
          user-select: none;
        }

        .chat-status {
          display: flex;
          align-items: center;
          gap: 8px;
        }

        .status-dot {
          width: 8px;
          height: 8px;
          background-color: var(--primary);
          border-radius: 50%;
          box-shadow: 0 0 5px var(--primary);
        }

        .chat-title {
          font-weight: bold;
          font-size: 14px;
          color: var(--primary);
          letter-spacing: 0.5px;
        }

        /* --- Controls (Buttons) --- */
        .chat-controls {
          display: flex;
          gap: 8px;
          align-items: center;
        }

        .control-btn {
          width: 32px;
          height: 32px;
          display: flex;
          align-items: center;
          justify-content: center;
          
          /* VISIBILITY FIX: Give it a faint green background by default */
          background: rgba(0, 255, 65, 0.1); 
          border: 1px solid var(--border);
          color: var(--primary);
          border-radius: 8px;
          cursor: pointer;
          transition: all 0.2s ease;
          padding: 0;
        }

        .control-btn:hover {
          background: var(--primary);
          color: #000;
          box-shadow: 0 0 8px var(--primary-dim);
        }

        /* Specific style for Close button on hover (Red) */
        .control-btn.close:hover {
          background: rgba(255, 60, 60, 0.2);
          border-color: #ff4444;
          color: #ff4444;
          box-shadow: 0 0 8px rgba(255, 60, 60, 0.2);
        }
        /* --- Body --- */
        .chat-body {
          flex: 1;
          height: 500px; /* Taller default height */
          background-color: var(--bg-dark);
        }

        /* --- States --- */
        .minimized {
          height: auto;
          width: 250px;
        }
        
        /* When minimized, just show the header, no body */
        .minimized .chat-header {
          border-bottom: none;
          padding: 10px;
        }

        /* --- Mobile --- */
        @media (max-width: 480px) {
          .chat-window {
            width: calc(100vw - 40px);
            right: 20px;
            bottom: 20px;
            max-height: 80vh;
          }
          
          .chat-body {
            height: 60vh;
          }
        }
      `}</style>
    </div>
  );
};

export default FloatingChatWidget;