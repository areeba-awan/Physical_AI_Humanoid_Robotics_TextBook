"use client";

import { useState, useRef, useEffect, useCallback } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { Bot, X, Send, Trash2, Loader2, ChevronDown, FileText, Minimize2, Maximize2 } from "lucide-react";
import ReactMarkdown from "react-markdown";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { oneDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import { Button } from "@/components/ui/button";
import { useChatStore } from "@/store/chat-store";
import { useAuthStore } from "@/store/auth-store";
import { chatApi } from "@/lib/api";
import { cn } from "@/lib/utils";
import type { ChatMessage, Citation } from "@/types/chat";

interface ChatWidgetProps {
  position?: "bottom-right" | "bottom-left";
  contextChapter?: string;
}

export function ChatWidget({ position = "bottom-right", contextChapter }: ChatWidgetProps) {
  const [input, setInput] = useState("");
  const [isMinimized, setIsMinimized] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const {
    messages,
    isOpen,
    isLoading,
    selectedText,
    toggleOpen,
    setOpen,
    addMessage,
    setLoading,
    setSelectedText,
    setContextChapter,
    clearHistory,
  } = useChatStore();

  const { session } = useAuthStore();

  // Set context chapter when prop changes
  useEffect(() => {
    if (contextChapter) {
      setContextChapter(contextChapter);
    }
  }, [contextChapter, setContextChapter]);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
      }
    };

    document.addEventListener("mouseup", handleSelection);
    return () => document.removeEventListener("mouseup", handleSelection);
  }, [setSelectedText]);

  const handleSend = useCallback(async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput("");

    // Add user message
    addMessage({
      role: "user",
      content: userMessage,
      contextChapter: contextChapter || undefined,
      selectedText: selectedText || undefined,
    });

    setLoading(true);

    try {
      if (session?.token) {
        const response: any = selectedText
          ? await chatApi.sendWithContext(session.token, {
              message: userMessage,
              selectedText,
              contextChapter,
            })
          : await chatApi.send(session.token, {
              message: userMessage,
              contextChapter,
            });

        addMessage({
          role: "assistant",
          content: response.content,
          sources: response.sources,
        });
      } else {
        // Demo response for non-authenticated users
        setTimeout(() => {
          addMessage({
            role: "assistant",
            content: "To use the AI assistant, please sign in to your account. You'll get access to personalized answers based on the textbook content.",
          });
        }, 500);
      }
    } catch (error) {
      addMessage({
        role: "assistant",
        content: "Sorry, I encountered an error. Please try again.",
      });
    } finally {
      setLoading(false);
      setSelectedText(null);
    }
  }, [input, isLoading, session, selectedText, contextChapter, addMessage, setLoading, setSelectedText]);

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const positionClasses = {
    "bottom-right": "right-4 bottom-4",
    "bottom-left": "left-4 bottom-4",
  };

  return (
    <div className={cn("fixed z-50", positionClasses[position])}>
      <AnimatePresence>
        {isOpen && (
          <motion.div
            initial={{ opacity: 0, y: 20, scale: 0.95 }}
            animate={{ opacity: 1, y: 0, scale: 1 }}
            exit={{ opacity: 0, y: 20, scale: 0.95 }}
            transition={{ duration: 0.2 }}
            className={cn(
              "bg-gradient-to-br from-indigo-50 to-purple-50 border-2 border-indigo-200 rounded-xl shadow-2xl mb-4 flex flex-col",
              isMinimized ? "w-80 h-14" : "w-96 h-[500px]"
            )}
          >
            {/* Header */}
            <div className="flex items-center justify-between px-4 py-3 border-b bg-gradient-to-r from-indigo-600 to-purple-600 text-white rounded-t-xl">
              <div className="flex items-center gap-2">
                <Bot className="h-5 w-5 text-white" />
                <span className="font-semibold">AI Assistant</span>
              </div>
              <div className="flex items-center gap-1">
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-7 w-7 text-white hover:bg-white/20"
                  onClick={() => setIsMinimized(!isMinimized)}
                >
                  {isMinimized ? (
                    <Maximize2 className="h-4 w-4" />
                  ) : (
                    <Minimize2 className="h-4 w-4" />
                  )}
                </Button>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-7 w-7 text-white hover:bg-white/20"
                  onClick={clearHistory}
                >
                  <Trash2 className="h-4 w-4" />
                </Button>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-7 w-7 text-white hover:bg-white/20"
                  onClick={() => setOpen(false)}
                >
                  <X className="h-4 w-4" />
                </Button>
              </div>
            </div>

            {!isMinimized && (
              <>
                {/* Selected Text Context */}
                {selectedText && (
                  <div className="px-4 py-2 bg-indigo-100 border-b border-indigo-200 flex items-center gap-2">
                    <FileText className="h-4 w-4 text-indigo-600 flex-shrink-0" />
                    <p className="text-xs text-indigo-800 truncate">
                      Context: "{selectedText.slice(0, 50)}..."
                    </p>
                    <Button
                      variant="ghost"
                      size="icon"
                      className="h-5 w-5 ml-auto flex-shrink-0 text-indigo-600 hover:bg-indigo-200"
                      onClick={() => setSelectedText(null)}
                    >
                      <X className="h-3 w-3" />
                    </Button>
                  </div>
                )}

                {/* Messages */}
                <div className="flex-1 overflow-y-auto p-4 space-y-4">
                  {messages.length === 0 ? (
                    <div className="text-center text-indigo-600 py-8">
                      <Bot className="h-12 w-12 mx-auto mb-4 opacity-70" />
                      <p className="text-sm">Ask me anything about the textbook!</p>
                      <p className="text-xs mt-2 text-indigo-500">
                        Tip: Select text on the page to ask about specific content.
                      </p>
                    </div>
                  ) : (
                    messages.map((message) => (
                      <ChatMessageBubble key={message.id} message={message} />
                    ))
                  )}

                  {isLoading && (
                    <div className="flex items-center gap-2 text-indigo-600">
                      <Loader2 className="h-4 w-4 animate-spin" />
                      <span className="text-sm">Thinking...</span>
                    </div>
                  )}

                  <div ref={messagesEndRef} />
                </div>

                {/* Input */}
                <div className="p-4 border-t border-indigo-200 bg-indigo-50">
                  <div className="flex items-end gap-2">
                    <textarea
                      ref={inputRef}
                      value={input}
                      onChange={(e) => setInput(e.target.value)}
                      onKeyDown={handleKeyDown}
                      placeholder="Ask a question..."
                      className="flex-1 min-h-[40px] max-h-[120px] resize-none rounded-lg border border-indigo-300 bg-white px-4 py-3 text-sm ring-offset-background placeholder:text-indigo-400 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-indigo-500 focus-visible:ring-offset-2"
                      rows={1}
                      disabled={isLoading}
                    />
                    <Button
                      size="icon"
                      onClick={handleSend}
                      disabled={!input.trim() || isLoading}
                      className="bg-indigo-600 hover:bg-indigo-700"
                    >
                      <Send className="h-4 w-4" />
                    </Button>
                  </div>
                </div>
              </>
            )}
          </motion.div>
        )}
      </AnimatePresence>

      {/* Toggle Button */}
      <motion.button
        whileHover={{ scale: 1.05 }}
        whileTap={{ scale: 0.95 }}
        onClick={toggleOpen}
        className={cn(
          "w-16 h-16 rounded-full shadow-2xl flex items-center justify-center transition-colors",
          isOpen
            ? "bg-gradient-to-r from-indigo-500 to-purple-500 text-white"
            : "bg-gradient-to-r from-indigo-600 to-purple-600 text-white hover:from-indigo-700 hover:to-purple-700"
        )}
      >
        {isOpen ? (
          <ChevronDown className="h-6 w-6" />
        ) : (
          <Bot className="h-6 w-6" />
        )}
      </motion.button>
    </div>
  );
}

function ChatMessageBubble({ message }: { message: ChatMessage }) {
  const isUser = message.role === "user";

  return (
    <div
      className={cn(
        "flex gap-3",
        isUser ? "justify-end" : "justify-start"
      )}
    >
      {!isUser && (
        <div className="w-8 h-8 rounded-full bg-indigo-100 flex items-center justify-center flex-shrink-0">
          <Bot className="h-4 w-4 text-indigo-600" />
        </div>
      )}

      <div
        className={cn(
          "max-w-[80%] rounded-xl px-4 py-3",
          isUser
            ? "bg-gradient-to-r from-indigo-500 to-purple-500 text-white"
            : "bg-gradient-to-r from-indigo-100 to-purple-100 text-indigo-900 border border-indigo-200"
        )}
      >
        <div className="prose prose-sm dark:prose-invert max-w-none">
          <ReactMarkdown
            components={{
              code({ node, className, children, ...props }: any) {
                const match = /language-(\w+)/.exec(className || "");
                const inline = !match;
                return !inline ? (
                  <SyntaxHighlighter
                    style={oneDark}
                    language={match[1]}
                    PreTag="div"
                    className="rounded-md text-xs"
                  >
                    {String(children).replace(/\n$/, "")}
                  </SyntaxHighlighter>
                ) : (
                  <code className={cn("bg-indigo-200/30 px-1 rounded text-indigo-800", className)} {...props}>
                    {children}
                  </code>
                );
              },
            }}
          >
            {message.content}
          </ReactMarkdown>
        </div>

        {/* Sources */}
        {message.sources && message.sources.length > 0 && (
          <div className="mt-2 pt-2 border-t border-indigo-200/50">
            <p className="text-xs text-indigo-600 mb-1">Sources:</p>
            <div className="flex flex-wrap gap-1">
              {message.sources.map((source, idx) => (
                <SourceBadge key={idx} source={source} />
              ))}
            </div>
          </div>
        )}
      </div>

      {isUser && (
        <div className="w-8 h-8 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 flex items-center justify-center flex-shrink-0">
          <span className="text-xs text-white font-medium">You</span>
        </div>
      )}
    </div>
  );
}

function SourceBadge({ source }: { source: Citation }) {
  return (
    <a
      href={`/book/${source.chapterId}`}
      className="inline-flex items-center gap-1 px-2 py-0.5 bg-indigo-200/30 rounded-lg text-xs text-indigo-700 hover:bg-indigo-300/50 transition-colors border border-indigo-300/50"
    >
      <FileText className="h-3 w-3" />
      <span>{source.section}</span>
    </a>
  );
}
