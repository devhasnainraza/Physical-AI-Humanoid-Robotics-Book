
import React, { useState, useEffect, useRef } from "react";
import { GoogleGenerativeAI } from "@google/generative-ai";
import ReactMarkdown from "react-markdown";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus } from "react-syntax-highlighter/dist/esm/styles/prism";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useUser } from "@clerk/clerk-react";
import getDb from "../../lib/db";
import {
  Mic, Volume2, VolumeX, Maximize2, Minimize2,
  Trash2, X, Send, Copy, Check, FileText, Lightbulb,
  Zap, BookOpen, User as UserIcon, Loader2,
  MoreVertical, Star, ThumbsUp, ThumbsDown, MicOff,
  BotMessageSquare, Sparkles
} from "lucide-react";

// --- Types & Theme ---
type Message = { 
  id: string; 
  role: 'user' | 'model'; 
  content: string;
  timestamp: number;
  isStreaming?: boolean;
};

const THEME = {
  primary: 'from-emerald-500 to-teal-500',
  glass: 'bg-white/80 dark:bg-slate-900/90 backdrop-blur-x1 saturate-150',
  border: 'border-slate-200/60 dark:border-slate-700/60',
  userBubble: 'bg-gradient-to-r from-emerald-500 to-teal-500',
  modelBubble: 'bg-white/90 dark:bg-slate-800/90',
  userText: 'text-white',
  modelText: 'text-slate-800 dark:text-slate-100'
};

// --- IMPROVED CHAT WIDGET ---
export default function ChatWidget() {
  const { siteConfig } = useDocusaurusContext();
  const { user } = useUser();
  
  // State
  const [isOpen, setIsOpen] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [context, setContext] = useState("");
  const [copiedId, setCopiedId] = useState<string | null>(null);
  const [feedback, setFeedback] = useState<Record<string, 'like' | 'dislike' | null>>({});
  
  // Voice input state
  const [isListening, setIsListening] = useState(false);
  const [recognition, setRecognition] = useState<any>(null);
  const [isSpeechSupported, setIsSpeechSupported] = useState(true);

  // Text selection state
  const [selectedText, setSelectedText] = useState<string>("");
  const [buttonPosition, setButtonPosition] = useState<{ top: number; left: number } | null>(null);
  const [showAskButton, setShowAskButton] = useState(false);

  // 🆕 SCROLLING STATE & REFS
  const messagesContainerRef = useRef<HTMLDivElement>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const [isUserNearBottom, setIsUserNearBottom] = useState(true);
  const [hasNewMessages, setHasNewMessages] = useState(false); // ✨ for enhanced button

  const neonUrl = siteConfig?.customFields?.neonConnectionString as string;
  const [speakingId, setSpeakingId] = useState<string | null>(null);

  // 🔒 SAFE SET INPUT
  const safeSetInput = (value: any) => {
    const safeValue = (value ?? "").toString().trim();
    setInput(safeValue);
  };

  // Initialize speech recognition
  useEffect(() => {
    if (typeof window !== "undefined") {
      const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
      if (SpeechRecognition) {
        const recognitionInstance = new SpeechRecognition();
        recognitionInstance.continuous = false;
        recognitionInstance.interimResults = true;
        recognitionInstance.lang = "en-US";

        recognitionInstance.onresult = (event: any) => {
          let transcript = "";
          for (let i = 0; i < event.results.length; i++) {
            transcript += event.results[i][0].transcript;
          }
          safeSetInput(transcript);
        };

        recognitionInstance.onerror = (event: any) => {
          console.error("Speech recognition error", event.error);
          setIsListening(false);
        };

        recognitionInstance.onend = () => {
          setIsListening(false);
        };

        setRecognition(recognitionInstance);
      } else {
        setIsSpeechSupported(false);
      }
    }
  }, []);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = "auto";
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 150)}px`;
    }
  }, [input]);

  // --- DB Handlers ---
  const loadHistory = async () => {
    if (!user || !neonUrl) {
      setMessages([{ id: 'init', role: 'model', content: 'Hello! Please log in to see your chat history.', timestamp: Date.now() }]);
      return;
    }
    try {
      const sql = getDb(neonUrl);
      if (!sql) throw new Error("DB connection failed.");
      const data: any[] = await sql`SELECT id::text, role, content, timestamp FROM chat_messages WHERE user_id = ${user.id} ORDER BY timestamp ASC`;
      if (data.length > 0) setMessages(data.map(msg => ({ ...msg, timestamp: Number(msg.timestamp) })));
      else setMessages([{ id: 'init', role: 'model', content: `Welcome back, **${user.firstName}**!`, timestamp: Date.now() }]);
    } catch (e) { console.error("Neon Load Error:", e); }
  };

  const saveMessage = async (msg: Omit<Message, 'id'>) => {
    const safeContent = (msg.content || "").toString();
    const fullMessage = { ...msg, content: safeContent, id: Date.now().toString(), timestamp: Date.now() };
    setMessages(prev => [...prev, fullMessage]);
    if (user && neonUrl) {
      const sql = getDb(neonUrl);
      if (sql) await sql`INSERT INTO chat_messages (user_id, role, content, timestamp) VALUES (${user.id}, ${msg.role}, ${safeContent}, ${Date.now()})`;
    }
  };

  const updateMessageContent = (id: string, newContent: string) => {
    const safeContent = (newContent || "").toString();
    setMessages(prev => 
      prev.map(msg => 
        msg.id === id ? { ...msg, content: safeContent } : msg
      )
    );
  };

  const clearHistory = async () => {
    setMessages([]);
    if (user && neonUrl) {
      const sql = getDb(neonUrl);
      if (sql) await sql`DELETE FROM chat_messages WHERE user_id = ${user.id}`;
    }
  };

  // --- Context Extraction ---
  useEffect(() => {
    const updateContext = () => {
      try {
        const contentDiv = document.querySelector('main');
        if (contentDiv) {
          const textContent = `${contentDiv.textContent || ""}`.substring(0, 30000);
          setContext(textContent);
        } else {
          setContext("");
        }
      } catch (e) {
        console.error("Context extraction error:", e);
        setContext("");
      }
    };
    setTimeout(updateContext, 1000);
  }, []);

  // --- 🆕 SCROLLING LOGIC ---
  useEffect(() => {
    const container = messagesContainerRef.current;
    if (!container) return;

    const handleScroll = () => {
      const { scrollTop, scrollHeight, clientHeight } = container;
      const isNearBottom = scrollHeight - scrollTop - clientHeight < 100;
      setIsUserNearBottom(isNearBottom);
    };

    container.addEventListener("scroll", handleScroll, { passive: true });
    handleScroll();

    return () => container.removeEventListener("scroll", handleScroll);
  }, []);

  // Track if there are unread new messages
  useEffect(() => {
    if (!isUserNearBottom && messages.length > 0) {
      setHasNewMessages(true);
    } else {
      setHasNewMessages(false);
    }
  }, [messages, isUserNearBottom]);

  // Auto-scroll when near bottom
  useEffect(() => {
    if (isUserNearBottom && messagesEndRef.current) {
      requestAnimationFrame(() => {
        messagesEndRef.current?.scrollIntoView({ 
          behavior: "smooth", 
          block: "end" 
        });
      });
    }
  }, [messages, isUserNearBottom]);

  // Load history
  useEffect(() => { 
    if (user) loadHistory(); 
  }, [user]);

  // Text selection AI button
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      
      if (!selection || selection.isCollapsed || !selection.toString().trim()) {
        setShowAskButton(false);
        return;
      }

      const rawText = selection.toString();
      const cleanText = (rawText || "").toString().trim();

      if (cleanText.length < 3) {
        setShowAskButton(false);
        return;
      }

      const anchorNode = selection.anchorNode;
      const focusNode = selection.focusNode;
      const isInsideWidget = (node: Node | null) => node?.parentElement?.closest?.('[data-chat-widget]');

      if (isInsideWidget(anchorNode) || isInsideWidget(focusNode)) {
        setShowAskButton(false);
        return;
      }

      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      const scrollX = window.scrollX || window.pageXOffset;
      const scrollY = window.scrollY || window.pageYOffset;

      let top = rect.top + scrollY - 45;
      let left = rect.left + scrollX + (rect.width / 2);

      if (rect.top < 50) {
        top = rect.bottom + scrollY + 10;
      }

      setSelectedText(cleanText);
      setButtonPosition({ top, left });
      setShowAskButton(true);
    };

    let timeoutId: any;
    const debouncedHandleSelection = () => {
      clearTimeout(timeoutId);
      timeoutId = setTimeout(handleSelection, 100);
    };

    const handleClear = () => setShowAskButton(false);

    document.addEventListener('mouseup', debouncedHandleSelection);
    document.addEventListener('touchend', debouncedHandleSelection);
    document.addEventListener('keyup', (e) => { if (e.key === 'Escape') handleClear(); });
    document.addEventListener('scroll', handleClear, { capture: true, passive: true });

    return () => {
      document.removeEventListener('mouseup', debouncedHandleSelection);
      document.removeEventListener('touchend', debouncedHandleSelection);
      document.removeEventListener('keyup', handleClear);
      document.removeEventListener('scroll', handleClear);
      clearTimeout(timeoutId);
    };
  }, []);

  // --- Voice Input Handlers ---
  const startListening = () => {
    if (recognition && !isListening) {
      recognition.start();
      setIsListening(true);
    }
  };

  const stopListening = () => {
    if (recognition && isListening) {
      recognition.stop();
      setIsListening(false);
    }
  };

  // --- Core Logic ---
  const handleSubmit = async (overrideMsg?: string) => {
    let finalMsg = "";
    if (overrideMsg != null && typeof overrideMsg === 'string' && overrideMsg.trim()) {
      finalMsg = overrideMsg.trim();
    } else if (input != null) {
      finalMsg = typeof input === 'string' ? input.trim() : String(input).trim();
    }

    if (!finalMsg) return;

    saveMessage({ role: "user", content: finalMsg });
    safeSetInput("");
    
    if (!isOpen) setIsOpen(true);
    setShowAskButton(false);

    const apiKey = siteConfig.customFields?.geminiApiKey as string;
    if (!apiKey) {
      saveMessage({ role: "model", content: "API Key not configured.", timestamp: Date.now() });
      return;
    }

    try {
      const genAI = new GoogleGenerativeAI(apiKey);
      const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash" });

      const prompt = `Context: "${context || ""}"\n\nQuestion: "${finalMsg}"`;

      const streamingMessageId = `streaming-${Date.now()}`;
      setMessages(prev => [
        ...prev, 
        { 
          id: streamingMessageId, 
          role: "model", 
          content: "", 
          timestamp: Date.now(),
          isStreaming: true
        }
      ]);

      const result = await model.generateContentStream(prompt);
      let fullText = "";
      
      for await (const chunk of result.stream) {
        try {
          const chunkText = chunk.text();
          const safeChunk = (chunkText || "").toString();
          fullText += safeChunk;
          updateMessageContent(streamingMessageId, fullText);
        } catch (chunkError) {
          console.error("Chunk processing error:", chunkError);
        }
      }

      const safeFullText = (fullText || "").toString();
      
      setMessages(prev => 
        prev.map(msg => 
          msg.id === streamingMessageId 
            ? { ...msg, isStreaming: false, content: safeFullText } 
            : msg
        )
      );

      if (user && neonUrl) {
        const sql = getDb(neonUrl);
        if (sql) await sql`INSERT INTO chat_messages (user_id, role, content, timestamp) VALUES (${user.id}, 'model', ${safeFullText}, ${Date.now()})`;
      }
    } catch (error: any) {
      setMessages(prev => prev.filter(msg => !msg.isStreaming));
      saveMessage({ role: "model", content: `❌ Error: ${error.message}`, timestamp: Date.now() });
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  const copyToClipboard = (text: string, id: string) => {
    navigator.clipboard.writeText(text);
    setCopiedId(id);
    setTimeout(() => setCopiedId(null), 2000);
  };

  const handleFeedback = (messageId: string, type: "like" | "dislike") => {
    setFeedback(prev => ({
      ...prev,
      [messageId]: prev[messageId] === type ? null : type
    }));
  };

  const speakMessage = (text: string, id: string) => {
    if (speakingId === id) {
      window.speechSynthesis.cancel();
      setSpeakingId(null);
    } else {
      window.speechSynthesis.cancel();
      const utterance = new SpeechSynthesisUtterance(text);
      utterance.onend = () => setSpeakingId(null);
      window.speechSynthesis.speak(utterance);
      setSpeakingId(id);
    }
  };

  const QuickActions = () => {
    const actions = [
      { label: "Summarize", icon: <FileText size={16} />, action: "Summarize this content" },
      { label: "Quiz", icon: <Lightbulb size={16} />, action: "Create a quiz based on this" },
      { label: "Explain", icon: <BookOpen size={16} />, action: "Explain this in simple terms" },
      { label: "In Urdu", icon: <Zap size={16} />, action: "Translate into Urdu" },
    ];

    return (
      <div className="flex gap-2 overflow-x-auto pb-3 scrollbar-hide">
        {actions.map((action) => (
          <button 
            key={action.label}
            onClick={() => handleSubmit(action.action)}
            className="flex items-center gap-2 px-3 py-1.5 bg-white/60 dark:bg-slate-800/60 border border-slate-200/50 dark:border-slate-700 rounded-full text-xs font-bold text-emerald-600 dark:text-emerald-400 hover:bg-white transition-all"
          >
            {action.icon}
            {action.label}
          </button>
        ))}
      </div>
    );
  };
  
  // --- Header Component ---
  const Header = () => (
    <div className="relative z-10 px-4 py-3 flex items-center justify-between border-b border-white/20 dark:border-slate-700/50 backdrop-blur-md bg-white/30 dark:bg-slate-900/30">
      <div className="flex items-center gap-3">
        <img src="https://devhasnainraza.github.io/Cortex-H1/img/CortexLogo.svg" alt="Cortex Logo" className="w-10 h-10" />
        <div>
          <h3 className="font-bold text-sm dark:text-white text-slate-800">Cortex Assistant</h3>
          <p className="text-xs text-slate-500 dark:text-slate-400">
            {user ? `${user.firstName} ${user.lastName || ""}` : "Guest"}
          </p>
        </div>
      </div>
      
      <div className="flex items-center gap-1">
        <button 
          onClick={() => { if(confirm("Clear all chat history?")) clearHistory(); }} 
          className="p-2 text-slate-500 hover:bg-red-100 dark:hover:bg-red-900/20 hover:text-red-500 rounded-lg transition-all active:scale-95"
          title="Clear History"
        >
          <Trash2 size={16}/>
        </button>
        <button 
          onClick={() => setIsExpanded(!isExpanded)} 
          className="p-2 text-slate-500 hover:bg-slate-100 dark:hover:bg-slate-700 rounded-lg transition-all active:scale-95"
          title={isExpanded ? "Minimize" : "Expand"}
        >
          {isExpanded ? <Minimize2 size={16}/> : <Maximize2 size={16}/>}
        </button>
        <button 
          onClick={() => setIsOpen(false)} 
          className="p-2 text-slate-500 hover:bg-red-100 dark:hover:bg-red-900/20 hover:text-red-500 rounded-lg transition-all active:scale-95"
          title="Close"
        >
          <X size={18}/>
        </button>
      </div>
    </div>
  );

  // --- Render ---
  return (
    <>
      {/* 🚀 Ask AI Button */}
      {showAskButton && buttonPosition && (
        <div
          className="fixed z-[10000] bg-gradient-to-r from-emerald-500 to-teal-500 text-white px-3 py-2 rounded-full shadow-lg flex items-center gap-2 cursor-pointer hover:scale-105 transition-transform"
          style={{
            left: `${buttonPosition.left}px`,
            top: `${buttonPosition.top}px`,
            transform: 'translateX(-50%)'
          }}
          onMouseDown={(e) => e.preventDefault()}
          onClick={(e) => {
            e.stopPropagation();
            handleSubmit(`Explain this: "${selectedText}"`);
          }}
        >
          <BotMessageSquare size={16} />
          <span className="text-sm font-medium">Ask AI</span>
        </div>
      )}

      {/* Main Chat Trigger Button */}
      {!isOpen && (
        <button 
          onClick={() => setIsOpen(true)} 
          className={`fixed bottom-6 right-6 w-14 h-14 rounded-2xl bg-gradient-to-br ${THEME.primary} shadow-xl hover:scale-110 transition-all z-50 flex items-center justify-center group`}
        >
          <BotMessageSquare className="w-7 h-7 text-white" />
          <div className="absolute -top-2 -right-2 w-6 h-6 bg-red-500 rounded-full flex items-center justify-center text-xs text-white opacity-0 group-hover:opacity-100 transition-opacity">
            {messages.filter(m => m.role === "model" && !m.isStreaming).length}
          </div>
        </button>
      )}

      {/* CHAT WINDOW */}
      <div 
        data-chat-widget
        className={`fixed z-[9999] flex flex-col font-sans transition-all duration-300 ${isOpen ? "opacity-100 pointer-events-auto translate-y-0" : "opacity-0 pointer-events-none translate-y-10"} ${isExpanded ? "inset-0" : "bottom-4 right-4 w-[90vw] h-[85vh] md:w-[450px] md:max-h-[700px] rounded-[24px]"} shadow-2xl border ${THEME.border} overflow-hidden`}
      >
        <div className={`absolute inset-0 ${THEME.glass} z-0`}></div>
        <Header />
        
        {/* 🆕 SCROLLABLE MESSAGES CONTAINER */}
        <div 
          ref={messagesContainerRef}
          className="relative z-10 flex-1 overflow-y-auto p-4 space-y-4 scrollbar-thin pt-2"
        >
          {messages.length === 0 && (
            <div className="flex flex-col items-center justify-center h-full text-center p-4">
              <div className="mb-4 p-3 bg-emerald-100 dark:bg-emerald-900/30 rounded-full">
                <img src="https://devhasnainraza.github.io/Cortex-H1/img/CortexLogo.svg" alt="Cortex Logo" className="w-8 h-8" />
              </div>
              <h3 className="font-bold text-lg mb-2 text-white">How can I help you today?</h3>
              <p className="text-slate-500 dark:text-slate-400 mb-4">Ask me anything about this page or related topics</p>
              <div className="grid grid-cols-2 gap-3 w-full max-w-xs">
                <button 
                  onClick={() => handleSubmit("What is this page about?")}
                  className="py-2 px-3 text-white bg-white dark:bg-slate-800 rounded-lg text-sm border border-slate-200 dark:border-slate-700 hover:bg-slate-50 dark:hover:bg-slate-700/50 transition-colors"
                >
                  Explain this
                </button>
                <button 
                  onClick={() => handleSubmit("Give me key points")}
                  className="py-2 px-3 text-white bg-white dark:bg-slate-800 rounded-lg text-sm border border-slate-200 dark:border-slate-700 hover:bg-slate-50 dark:hover:bg-slate-700/50 transition-colors"
                >
                  Summarize
                </button>
              </div>
            </div>
          )}

          {messages.map((m) => (
            <div 
              key={m.id} 
              className={`flex w-full gap-3 transition-all duration-300 ${m.role === "user" ? "justify-end" : "justify-start"}`}
            >
              {m.role === "model" && (
                <div className="w-8 h-8 rounded-full bg-gradient-to-br flex items-center justify-center shrink-0 mt-1">
                  <img src="https://devhasnainraza.github.io/Cortex-H1/img/CortexLogo.svg" alt="Cortex Logo" className="w-9 h-9" />
                </div>
              )}
              
              <div className={`max-w-[85%] group relative ${m.role === "user" ? "order-first" : ""}`}>
                <div 
                  className={`px-4 py-3 rounded-[20px] shadow-sm border ${
                    m.role === "user" 
                      ? `${THEME.userBubble} ${THEME.userText} border-emerald-500/30` 
                      : `${THEME.modelBubble} ${THEME.modelText} border-slate-200/50 dark:border-slate-700/50`
                  } transition-all duration-200 hover:shadow-md`}
                >
                  <ReactMarkdown 
                    components={{
                      code({node, inline, className, children, ...props}) {
                        const match = /language-(\w+)/.exec(className || "");
                        return !inline && match ? (
                          <SyntaxHighlighter
                            style={vscDarkPlus}
                            language={match[1]}
                            PreTag="div"
                            {...props}
                            className="rounded-lg my-2"
                          >
                            {String(children).replace(/\n$/, "")}
                          </SyntaxHighlighter>
                        ) : (
                          <code 
                            className={`px-1.5 py-0.5 rounded bg-slate-100 dark:bg-slate-700 text-sm ${m.role === "user" ? "dark:text-white" : "dark:text-slate-200"}`}
                            {...props}
                          >
                            {children}
                          </code>
                        );
                      },
                      p: ({node, ...props}) => <p className="mb-3 last:mb-0" {...props} />,
                      ul: ({node, ...props}) => <ul className="mb-3 pl-5 list-disc" {...props} />,
                      ol: ({node, ...props}) => <ol className="mb-3 pl-5 list-decimal" {...props} />,
                      li: ({node, ...props}) => <li className="mb-1" {...props} />,
                      a: ({node, href, ...props}) => (
                        <a 
                          href={href} 
                          target="_blank" 
                          rel="noopener noreferrer"
                          className={`underline ${m.role === "user" ? "text-emerald-200 hover:text-white" : "text-emerald-500 hover:text-emerald-600 dark:text-emerald-400 dark:hover:text-emerald-300"}`}
                          {...props}
                        />
                      )
                    }}
                  >
                    {m.content}
                  </ReactMarkdown>
                  
                  {m.role === "model" && !m.isStreaming && (
                    <div className="flex gap-1 mt-2 opacity-0 group-hover:opacity-100 transition-opacity">
                      <button 
                        onClick={() => copyToClipboard(m.content, m.id)}
                        className="p-1.5 rounded-lg hover:bg-slate-200/50 dark:hover:bg-slate-700/50 transition-colors"
                        title="Copy"
                      >
                        {copiedId === m.id ? <Check size={14} /> : <Copy size={14} />}
                      </button>
                      <button 
                        onClick={() => handleFeedback(m.id, "like")}
                        className={`p-1.5 rounded-lg transition-colors ${feedback[m.id] === "like" ? "text-green-500 bg-green-500/10" : "hover:bg-slate-200/50 dark:hover:bg-slate-700/50"}`}
                      >
                        <ThumbsUp size={14} />
                      </button>
                      <button 
                        onClick={() => handleFeedback(m.id, "dislike")}
                        className={`p-1.5 rounded-lg transition-colors ${feedback[m.id] === "dislike" ? "text-red-500 bg-red-500/10" : "hover:bg-slate-200/50 dark:hover:bg-slate-700/50"}`}
                      >
                        <ThumbsDown size={14} />
                      </button>
                      <button 
                        onClick={() => speakMessage(m.content, m.id)}
                        className={`p-1.5 rounded-lg transition-all ${speakingId === m.id ? "text-emerald-400 bg-emerald-500/20" : "hover:bg-slate-700/50"}`}
                      >
                        {speakingId === m.id ? <VolumeX size={14} /> : <Volume2 size={14} />}
                      </button>
                    </div>
                  )}
                  
                  {m.isStreaming && (
                    <div className="flex items-center mt-2">
                      <div className="flex space-x-1">
                        <div className="w-2 h-2 bg-emerald-500 rounded-full animate-bounce"></div>
                        <div className="w-2 h-2 bg-emerald-500 rounded-full animate-bounce delay-75"></div>
                        <div className="w-2 h-2 bg-emerald-500 rounded-full animate-bounce delay-150"></div>
                      </div>
                    </div>
                  )}
                </div>
                <div className={`text-xs mt-1 ${m.role === "user" ? "text-right" : "text-left"}`}>
                  <span className="text-slate-500 dark:text-slate-400">
                    {new Date(m.timestamp).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}
                  </span>
                </div>
              </div>
              
              {m.role === "user" && (
                <div className="w-8 h-8 rounded-full bg-slate-200 dark:bg-slate-700 flex items-center justify-center shrink-0 mt-1">
                  <UserIcon size={16} />
                </div>
              )}
            </div>
          ))}
          <div ref={messagesEndRef} />
        </div>

        {/* 🆕 ✨ ENHANCED SCROLL-TO-BOTTOM BUTTON */}
        {hasNewMessages && (
          <button
            onClick={() => {
              setHasNewMessages(false);
              messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
            }}
            className={`
              absolute bottom-24 right-4 z-20
              w-10 h-10 rounded-full
              bg-gradient-to-r from-emerald-500 to-teal-500
              text-white flex items-center justify-center
              shadow-lg shadow-emerald-500/30
              hover:shadow-emerald-500/50
              hover:scale-105
              active:scale-95
              transition-all duration-200
              animate-pulse
              group
            `}
            aria-label="New messages — click to scroll to bottom"
            title="New messages • Scroll down"
          >
            <Sparkles 
              size={18} 
              className="group-hover:rotate-12 transition-transform duration-300" 
            />
            {/* Subtle animated ring for attention */}
            <div className="absolute inset-0 rounded-full border-2 border-white/30 animate-ping opacity-20" />
          </button>
        )}

        <div className="relative z-10 p-3 border-t border-white/20 dark:border-slate-700/50 bg-white/50 dark:bg-slate-900/50 backdrop-blur-md">
          <QuickActions />
          <div className="relative flex items-end gap-2 mt-3 bg-white dark:bg-slate-800 p-2 rounded-[18px] shadow-lg">
            <textarea
              ref={textareaRef}
              value={input}
              onChange={(e) => safeSetInput(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask a question..."
              className="flex-1 bg-transparent border-none outline-none dark:text-white py-2 px-3 resize-none min-h-[44px] max-h-[150px]"
              rows={1}
            />
            
            {!isSpeechSupported ? (
              <div className="p-3 text-slate-400 cursor-not-allowed" title="Speech not supported">
                <MicOff size={20} />
              </div>
            ) : (
              <button
                onClick={isListening ? stopListening : startListening}
                className={`p-3 rounded-xl transition-all ${isListening ? "bg-emerald-500 text-white shadow-lg shadow-emerald-500/20 hover:scale-105 active:scale-95" : "bg-slate-100 dark:bg-slate-700 text-slate-500 dark:text-slate-300 hover:bg-slate-200 dark:hover:bg-slate-600"}`}
                title={isListening ? "Stop listening" : "Start voice input"}
              >
                {isListening ? <MicOff className="w-5 h-5" /> : <Mic className="w-5 h-5" />}
              </button>
            )}
            
            <button 
              onClick={() => handleSubmit()} 
              disabled={!input.trim()}
              className={`p-3 rounded-xl transition-all ${input.trim() ? `bg-gradient-to-r ${THEME.primary} text-white shadow-lg shadow-emerald-500/20 hover:scale-105 active:scale-95` : "bg-slate-100 dark:bg-slate-700 text-slate-300"}`}
            >
              <Send className="w-5 h-5" />
            </button>
          </div>
          <div className="flex justify-between mt-2 text-xs text-slate-500 dark:text-slate-400">
            <span>Press Enter to send, Shift+Enter for new line</span>
            <span>{input.length}/2000</span>
          </div>
        </div>
      </div>
    </>
  );
}
