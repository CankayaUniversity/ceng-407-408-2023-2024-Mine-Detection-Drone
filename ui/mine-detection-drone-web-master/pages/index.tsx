import React, { useState, useEffect } from "react";
import Head from "next/head";
import Image from "next/image";
import axios from "axios";

import BatteryLevel from "@/components/BatteryLevel";
import Coordinates from "@/components/Coordinates";
import DroneHeight from "@/components/DroneHeight";
import FlightTime from "@/components/FlightTime";
import Login from "@/components/Login";
import MineCoordinates from "@/components/MineCoordinates";
import MineDetection from "@/components/MineDetection";
import RotorsStates from "@/components/RotorsStates";

import camo from "../assets/camo.png";

interface BufferCoordinates {
  latitude: number;
  longitude: number;
}

export default function Home() {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [bufferCoordinates, setBufferCoordinates] = useState<BufferCoordinates | null>(null);

  const handleLogin = () => {
    setIsLoggedIn(true);
  };

  useEffect(() => {
    const fetchBufferCoordinates = async () => {
      try {
        const response = await axios.get(
          "http://localhost:5000/send-coordinates",
        );

        setBufferCoordinates(response.data.last_coordinates);
      } catch (error) {
        console.error("Error fetching coordinates: ", error);
      }
    };

    const interval = setInterval(fetchBufferCoordinates, 1000);
    fetchBufferCoordinates();

    return () => clearInterval(interval);
  }, []);

  return (
    <>
      <Head>
        <title>Mine Detection Drone UI</title>
      </Head>

      <div className="overflow-hidden w-full flex">
        <aside className="relative w-[384px] bg-repeat-y bg-gradient-to-r from-transparent to-background">
          <Image
            src={camo}
            className="absolute z-0 inset-0 object-cover"
            layout="fill"
            alt=""
          />
          <div className="absolute z-10 inset-0 bg-gradient-to-r from-transparent to-background"></div>
        </aside>

        <main className="flex-1 flex justify-center">
          {isLoggedIn ? (
            <div className="flex">
              <div className="mr-4">
                <MineDetection />
                <Coordinates />
              </div>

              <div className="mr-4 flex">
                <MineCoordinates addMineCoordinates={() => bufferCoordinates} />
              </div>

              <div>
                <FlightTime />
                <DroneHeight />
              </div>
            </div>
          ) : (
            <Login onLogin={handleLogin} />
          )}
        </main>

        <aside className="relative w-[384px] bg-repeat-y bg-gradient-to-l from-transparent to-background">
          <Image
            src={camo}
            className="absolute z-0 inset-0 object-cover"
            layout="fill"
            alt=""
          />
          <div className="absolute z-10 inset-0 bg-gradient-to-l from-transparent to-background"></div>
        </aside>
      </div>
    </>
  );
}
