import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom';
import { SignedIn, SignedOut, SignInButton, UserButton } from "@clerk/clerk-react";

export default function ClerkAuth() {
  const [navbarContainer, setNavbarContainer] = useState<Element | null>(null);

  useEffect(() => {
    const navRight = document.querySelector('.navbar__items--right');
    if (navRight) setNavbarContainer(navRight);
  }, []);

  if (!navbarContainer) return null;

  return ReactDOM.createPortal(
    <div className="flex items-center gap-3 mx-2">
        <SignedOut>
            <SignInButton mode="modal">
                <button className="px-4 py-1.5 bg-blue-600 hover:bg-blue-700 text-white rounded-lg text-xs font-bold transition-all shadow-lg">
                    Sign In
                </button>
            </SignInButton>
        </SignedOut>
        <SignedIn>
            <UserButton afterSignOutUrl="/" />
        </SignedIn>
    </div>,
    navbarContainer
  );
}
