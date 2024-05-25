import React, { useState, useEffect } from "react";
import Image from "next/image";
import axios from "axios";

import stopwatch from "../assets/stopwatch.png";

const FlightTime: React.FC = () => {
  const [flightTime, setFlightTime] = useState<number | null>(null);

  useEffect(() => {
    const fetchFlightTime = async () => {
      try {
        const response = await axios.get(
          "http://localhost:5000/send-flight-time",
        );

        setFlightTime(response.data.flight_time);
      } catch (error) {
        console.error("Error fetching flight time: ", error);
      }
    };

    const interval = setInterval(fetchFlightTime, 1000);
    fetchFlightTime();

    return () => clearInterval(interval);
  }, []);

  const formatTime = (time: number) => {
    const hours = Math.floor(time / 3600);
    const minutes = Math.floor((time % 3600) / 60);
    const seconds = time % 60;

    return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`;
  };

  return (
    <div className="my-8 ml-8 p-4 shadow-md shadow-rust rounded bg-window">
      <div className="py-3 px-8 flex justify-center items-center text-2xl font-bold">
        <p className="">Flight Time</p>
      </div>

      <div className="flex justify-center">
        <Image width="64" height="64" alt="" src={stopwatch} />
      </div>

      <div className="py-3 px-8 flex justify-center items-center font-semibold">
        {flightTime !== null ? (
          <p>Flight Time: {formatTime(flightTime)}</p>
        ) : (
          <p>Loading...</p>
        )}
      </div>
    </div>
  );
};

export default FlightTime;
