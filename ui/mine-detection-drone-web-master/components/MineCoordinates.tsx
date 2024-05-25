import React, { useState } from "react";
import Image from "next/image";

import mine from "../assets/mine.png";

interface MineCoordinates {
  latitude: number;
  longitude: number;
}

const MineCoordinates: React.FC<{ addMineCoordinates: () => MineCoordinates | null }> = ({
  addMineCoordinates,
}) => {
  const [mineCoordinatesList, setMineCoordinatesList] = useState<MineCoordinates[]>([]);

  const handleClick = () => {
    const newMineCoordinates = addMineCoordinates();
    if (newMineCoordinates) {
      setMineCoordinatesList((prevList) => [...prevList, newMineCoordinates]);
    }
  };

  return (
    <div className=" w-[512px] ml-4 my-8 p-6 rounded bg-window shadow-md shadow-rust">
      <div className="flex justify-center mt-8 mb-12">
        <Image width="64" height="64" alt="" src={mine} />

        <div className="flex justify-center items-center px-2 py-3 text-2xl font-bold">
          <p className="text-chrome">Found</p>&nbsp;<p>Mines</p>
        </div>
      </div>

      <div className="flex justify-center items-center">
        <button
          className="mb-8 px-36 py-3 rounded bg-rust hover:bg-metal font-semibold text-lg text-white"
          type="button"
          onClick={handleClick}
        >
          Add Location
        </button>
      </div>
      
      <ul>
        {mineCoordinatesList.map((coordinates, index) => (
          <li key={index} className="text-center">
            Latitude: {coordinates.latitude}, Longitude: {coordinates.longitude}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default MineCoordinates;
