import React, { useState, useEffect } from "react";
import Image from "next/image";
import axios from "axios";

import radar from "../assets/radar.png";

const MineDetection: React.FC = () => {
  const [mineDetection, setMineDetection] = useState<boolean | null>(null);

  useEffect(() => {
    const fetchMineDetection = async () => {
      try {
        const response = await axios.get(
          "http://localhost:5000/send-mine-detection",
        );

        setMineDetection(response.data.mine_detected);
      } catch (error) {
        console.error("Error fetching mine detection: ", error);
      }
    };

    const interval = setInterval(fetchMineDetection, 1000);
    fetchMineDetection();

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="my-8 mr-4 p-14 shadow-md shadow-rust rounded bg-window">
      <div className="flex justify-center">
        <Image width="64" height="64" alt="" src={radar} />
      </div>

      <div className="mt-8 px-8 flex justify-center items-center text-2xl font-bold">
        {mineDetection ? (
          <div className="text-invalid">
            <p>Mine Found!</p>
          </div>
        ) : (
          <div className="text-valid">
            <p>Scanning...</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default MineDetection;
