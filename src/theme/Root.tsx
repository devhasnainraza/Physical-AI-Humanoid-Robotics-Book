import React from "react";
import type { ReactNode } from "react";
import { ClerkProvider } from "@clerk/clerk-react";

// Read Clerk publishable key from env
// Docusaurus makes process.env available at build time
// For runtime, we can also check window if needed
const getClerkKey = () => {
  if (typeof process !== "undefined" && process.env?.CLERK_PUBLISHABLE_KEY) {
    return process.env.CLERK_PUBLISHABLE_KEY;
  }
  // Runtime fallback (if set via script tag or other means)
  if (typeof window !== "undefined") {
    return (window as any).CLERK_PUBLISHABLE_KEY || "";
  }
  return "";
};

const clerkPublishableKey = getClerkKey();

export default function Root(props: { children: ReactNode }): JSX.Element {
  // If no key is provided, render children without ClerkProvider
  // This allows the app to load even if Clerk isn't configured yet
  if (!clerkPublishableKey || clerkPublishableKey.trim() === "") {
    if (typeof window !== "undefined") {
      console.warn(
        "[Clerk] CLERK_PUBLISHABLE_KEY is not set. Please set it in your .env file as CLERK_PUBLISHABLE_KEY=pk_test_..."
      );
    }
    return <>{props.children}</>;
  }

  return (
    <ClerkProvider
      publishableKey={clerkPublishableKey}
      navigate={(to) => {
        if (typeof window !== "undefined") {
          window.location.href = to;
        }
      }}
    >
      {props.children}
    </ClerkProvider>
  );
}


