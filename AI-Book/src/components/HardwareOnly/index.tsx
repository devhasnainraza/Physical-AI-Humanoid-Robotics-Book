import React from 'react';
import { useHardwareProfile } from '../Auth/AuthProvider';

interface HardwareOnlyProps {
  profile: string;
  children: React.ReactNode;
}

export default function HardwareOnly({ profile, children }: HardwareOnlyProps) {
  const { hardwareProfile } = useHardwareProfile();

  // Show if user has specific profile OR if user has no profile (cloud simulation default) 
  // Wait, logic in spec: "Content inside these tags should only render if the logged-in user has that specific hardware profile."
  // So strict match.
  
  if (hardwareProfile === profile) {
    return <>{children}</>;
  }

  return null;
}
