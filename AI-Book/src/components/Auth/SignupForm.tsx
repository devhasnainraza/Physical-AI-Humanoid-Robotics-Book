import React, { useState } from 'react';
import { authClient } from '../../lib/auth-client'; // Assuming client library wrapper

export function SignupForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [hardwareProfile, setHardwareProfile] = useState('Cloud_Simulation_Only');

  const handleSubmit = async () => {
    await authClient.signUp.email({
      email,
      password,
      name,
      // @ts-ignore - extra fields
      hardware_profile: hardwareProfile,
    });
  };

  return (
    <div className="signup-form">
      <input 
        type="text" 
        placeholder="Name"
        value={name} 
        onChange={(e) => setName(e.target.value)} 
      />
      <input 
        type="email" 
        placeholder="Email"
        value={email} 
        onChange={(e) => setEmail(e.target.value)} 
      />
      <input 
        type="password" 
        placeholder="Password"
        value={password} 
        onChange={(e) => setPassword(e.target.value)} 
      />
      <select 
        value={hardwareProfile} 
        onChange={(e) => setHardwareProfile(e.target.value)}
      >
        <option value="Cloud_Simulation_Only">Cloud Simulation Only</option>
        <option value="Jetson_Orin_Kit">Jetson Orin Kit</option>
        <option value="Unitree_Go2_Robot">Unitree Go2 Robot</option>
      </select>
      <button onClick={handleSubmit}>Sign Up</button>
    </div>
  );
}
