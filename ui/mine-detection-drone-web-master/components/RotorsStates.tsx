import React, { useState, useEffect } from "react";
import Image from "next/image";
import axios from "axios";

import drone from "../assets/drone.png";

interface RotorsStates {
  rotor_nw: {
    rotor_nw_rpm: number;
    rotor_nw_angle: number;
  },
  rotor_ne: {
    rotor_ne_rpm: number;
    rotor_ne_angle: number;
  },
  rotor_sw: {
    rotor_sw_rpm: number;
    rotor_sw_angle: number;
  },
  rotor_se: {
    rotor_se_rpm: number;
    rotor_se_angle: number;
  };
}

const RotorsStatesComponent: React.FC = () => {
  const [rotorsStates, setRotorsStates] = useState<RotorsStates | null>(null);

  useEffect(() => {
    const fetchRotorsStates = async () => {
      try {
        const response = await axios.get(
          "http://localhost:5000/send-rotors-states"
        );

        setRotorsStates(response.data.rotors_states);
      } catch (error) {
        console.error("Error fetching rotors' states: ", error);
      }
    };

    const interval = setInterval(fetchRotorsStates, 1000);
    fetchRotorsStates();

    return () => clearInterval(interval);
  }, []);

  return (
    <div>
      <div className="my-8 ml-8 p-10 shadow-md shadow-rust rounded bg-window">
        <div className="mt-4 flex justify-center">
          <Image width="64" height="64" alt="" src={drone} />
        </div>

        <div className="mt-4 py-3 px-8 flex justify-center items-center text-2xl font-bold">
          <p>Rotors</p>&apos;&nbsp;<p>States</p>
        </div>

        {rotorsStates ? (
          <div className="mx-8">
            <div className="flex">
              <div>
                <p className="font-semibold">Rotor NW</p>
                <div className="">
                  <p>RPM = {rotorsStates.rotor_nw.rotor_nw_rpm}</p>
                  <p>Angle = {rotorsStates.rotor_nw.rotor_nw_angle}</p>
                </div>
              </div>
              <div>
                <p className="font-semibold">Rotor NE</p>
                <div className="">
                  <p>RPM = {rotorsStates.rotor_ne.rotor_ne_rpm}</p>
                  <p>Angle = {rotorsStates.rotor_ne.rotor_ne_angle}</p>
                </div>
              </div>
            </div>
            <div className="ml-auto flex">
              <div>
                <p className="font-semibold">Rotor SW</p>
                <div className="">
                  <p>RPM = {rotorsStates.rotor_sw.rotor_sw_rpm}</p>
                  <p>Angle = {rotorsStates.rotor_sw.rotor_sw_angle}</p>
                </div>
              </div>
              <div>
                <p className="font-semibold">Rotor SE</p>
                <div className="">
                  <p>RPM = {rotorsStates.rotor_se.rotor_se_rpm}</p>
                  <p>Angle = {rotorsStates.rotor_se.rotor_se_angle}</p>
                </div>
              </div>
            </div>
          </div>
        ) : (
          <p className="my-8 flex justify-center items-center text-xl font-semibold">
            Loading...
          </p>
        )}
      </div>
    </div>
  );
};

export default RotorsStatesComponent;
