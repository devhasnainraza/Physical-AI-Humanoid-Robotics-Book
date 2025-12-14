import React, { createContext, useContext } from 'react';
import { authClient } from '../../lib/auth-client';

const AuthContext = createContext<{ hardwareProfile: string | null }>({ hardwareProfile: null });

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const { data: session } = authClient.useSession();
  
  // @ts-ignore - Better Auth types might not auto-infer custom fields without declaration merging
  const hardwareProfile = session?.user?.hardware_profile || 'Cloud_Simulation_Only';

  return (
    <AuthContext.Provider value={{ hardwareProfile }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useHardwareProfile = () => useContext(AuthContext);
