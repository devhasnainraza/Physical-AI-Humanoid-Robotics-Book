import React from 'react';
import { useUser } from '@clerk/clerk-react';

interface HardwareOnlyProps {
  profile: string;
  children: React.ReactNode;
}

export default function HardwareOnly({ profile, children }: HardwareOnlyProps) {
  const { user, isLoaded } = useUser();

  if (!isLoaded) return null;

  // Default everyone to 'Cloud_Simulation_Only' if not set
  const userProfile = (user?.unsafeMetadata?.hardware_profile as string) || 'Cloud_Simulation_Only';

  if (userProfile === profile) {
    return <>{children}</>;
  }

  return null;
}
