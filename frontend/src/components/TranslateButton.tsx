import React, { useState } from 'react';

export default function TranslateButton() {
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Helper function to replace text content while preserving HTML structure
  const replaceTextContent = (element: HTMLElement, originalText: string, translatedText: string) => {
    // For elements with no child elements, we can safely set textContent
    if (element.children.length === 0) {
      element.textContent = translatedText;
      return;
    }

    // For elements with child elements, we need to replace text nodes carefully
    // without disrupting the DOM structure
    const walker = document.createTreeWalker(
      element,
      NodeFilter.SHOW_TEXT,
      null
    );

    const textNodes: Text[] = [];
    let node;
    while (node = walker.nextNode()) {
      textNodes.push(node as Text);
    }

    // Replace text in each text node that matches the original text
    textNodes.forEach(textNode => {
      // Only replace if the text node contains the exact original text
      if (textNode.textContent?.trim() === originalText.trim()) {
        textNode.textContent = translatedText;
      } else if (textNode.textContent?.includes(originalText)) {
        // If it contains the original text as a substring, replace just that part
        textNode.textContent = textNode.textContent?.replace(originalText, translatedText) || '';
      }
    });
  };

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

  const translate = async () => {
    if (isTranslating) return; // Prevent multiple clicks

    // Check if running in browser environment
    if (typeof window === 'undefined') {
      setError('Translation is only available in browser environment');
      return;
    }

    setIsTranslating(true);
    setError(null);

    try {
      // Get the article content
      const articleElement = document.querySelector('article');
      if (!articleElement) {
        throw new Error('Article content not found');
      }

      // Get all text content except code blocks
      const elements = articleElement.querySelectorAll('div, p, h1, h2, h3, h4, h5, h6, span, li');
      const contentToTranslate = Array.from(elements).map(el => ({
        element: el as HTMLElement, // Cast to HTMLElement to access style properties
        originalText: el.textContent || '',
        isCode: el.closest('pre, code') !== null
      })).filter(item => item.originalText.trim() !== '' && !item.isCode);

      if (contentToTranslate.length === 0) {
        throw new Error('No translatable content found');
      }

      // Show a warning to the user
      if (!confirm('This will translate the content to Urdu. Auto-translated content may have errors. Continue?')) {
        setIsTranslating(false);
        return;
      }

      // Translate each piece of content
      for (const item of contentToTranslate) {
        if (item.originalText.trim() !== '' && !item.isCode) {
          // Call the backend translation API with dynamic API base
          const apiBase = getApiBase();
          const res = await fetch(`${apiBase}/api/translate-text`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              text: item.originalText,
              target_language: 'ur'
            }),
          });

          if (!res.ok) {
            throw new Error(`Translation API error: ${res.status}`);
          }

          const { translated_text } = await res.json();

          // Apply proper Urdu formatting with right-to-left direction
          item.element.setAttribute('dir', 'rtl');
          item.element.style.direction = 'rtl'; // Right-to-left for Urdu
          item.element.style.textAlign = 'right';
          item.element.style.fontFamily = 'Tahoma, Arial, sans-serif'; // Better font support for Urdu

          // Preserve the element's structure by only replacing text nodes
          replaceTextContent(item.element, item.originalText, translated_text);
        }
      }

      alert('Translation to Urdu completed!');
    } catch (err) {
      console.error('Translation error:', err);
      setError(err instanceof Error ? err.message : 'An error occurred during translation');
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div className="translate-button-container" style={{ marginBottom: '20px' }}>
      <button
        onClick={translate}
        disabled={isTranslating}
        style={{
          padding: '8px 16px',
          backgroundColor: isTranslating ? '#ccc' : '#00cc44',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isTranslating ? 'not-allowed' : 'pointer',
        }}
      >
        {isTranslating ? 'Translating...' : '.Translate to Urdu'}
      </button>
      {error && (
        <div style={{ color: 'red', marginTop: '5px' }}>
          Error: {error}
        </div>
      )}
    </div>
  );
}