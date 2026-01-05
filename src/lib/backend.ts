const normalizeUrl = (url: string) => url.replace(/\/$/, "");

const readWindowValue = (key: string) => {
  if (typeof window === "undefined") return undefined;
  const win = window as any;
  return win?.[key] as string | undefined;
};

const readMetaTag = (name: string) => {
  if (typeof document === "undefined") return undefined;
  const meta = document.querySelector(`meta[name="${name}"]`);
  return meta?.getAttribute("content") ?? undefined;
};

export const getBackendUrl = () => {
  const envUrl =
    (typeof process !== "undefined" && process.env?.REACT_APP_BACKEND_URL) ||
    (typeof process !== "undefined" && process.env?.NEXT_PUBLIC_BACKEND_URL) ||
    (typeof process !== "undefined" && process.env?.BACKEND_URL);

  const windowUrl =
    readWindowValue("__RAG_BACKEND_URL") || readWindowValue("__BACKEND_URL");

  const metaUrl =
    readMetaTag("rag-backend-url") || readMetaTag("backend-url") || undefined;

  const fallback = "http://localhost:8000";

  const chosen = envUrl || windowUrl || metaUrl || fallback;
  return normalizeUrl(chosen);
};

export const TOKEN_STORAGE_KEY = "rag_jwt_token";
export const EMAIL_STORAGE_KEY = "rag_auth_email";

export const getStoredAuth = () => {
  if (typeof window === "undefined") return { token: null, email: null };
  return {
    token: localStorage.getItem(TOKEN_STORAGE_KEY),
    email: localStorage.getItem(EMAIL_STORAGE_KEY),
  };
};

export const persistAuth = (token?: string, email?: string) => {
  if (typeof window === "undefined") return;
  if (token) {
    localStorage.setItem(TOKEN_STORAGE_KEY, token);
  } else {
    localStorage.removeItem(TOKEN_STORAGE_KEY);
  }

  if (email) {
    localStorage.setItem(EMAIL_STORAGE_KEY, email);
  } else {
    localStorage.removeItem(EMAIL_STORAGE_KEY);
  }
};

export const clearAuth = () => {
  if (typeof window === "undefined") return;
  localStorage.removeItem(TOKEN_STORAGE_KEY);
  localStorage.removeItem(EMAIL_STORAGE_KEY);
};

export const buildAuthHeaders = (token?: string) =>
  token ? { Authorization: `Bearer ${token}` } : {};

