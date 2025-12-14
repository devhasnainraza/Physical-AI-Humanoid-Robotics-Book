export enum HardwareProfile {
  Cloud_Simulation_Only = "Cloud_Simulation_Only",
  Jetson_Orin_Kit = "Jetson_Orin_Kit",
  Unitree_Go2_Robot = "Unitree_Go2_Robot",
}

export type UserHardwareProfile = keyof typeof HardwareProfile;
