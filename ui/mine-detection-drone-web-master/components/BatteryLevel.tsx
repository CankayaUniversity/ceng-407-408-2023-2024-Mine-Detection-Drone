import React, { useState, useEffect } from "react";
import Image from "next/image";
import axios from "axios";

import battery from "../assets/battery.png";

const BatteryLevel: React.FC = () => {
  const [batteryLevel, setBatteryLevel] = useState<number | null>(null);

  useEffect(() => {
    const fetchBatteryLevel = async () => {
      try {
        const response = await axios.get(
          "http://localhost:5000/send-battery-level",
        );

        setBatteryLevel(response.data.battery_level);
      } catch (error) {
        console.error("Error fetching battery level: ", error);
      }
    };

    const interval = setInterval(fetchBatteryLevel, 10000);
    fetchBatteryLevel();

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="my-8 ml-8 p-4 shadow-md shadow-rust rounded bg-window">
      <div className="mt-4 mb-2 flex justify-center">
        <Image width="64" height="64" alt="" src={battery} />
        {batteryLevel !== null ? (
          <p className="mt-5 ml-4 font-semibold">{batteryLevel}%</p>
        ) : (
          <p className="mt-5 ml-4 font-semibold">Loading...</p>
        )}
      </div>
    </div>
  );
};

export default BatteryLevel;
