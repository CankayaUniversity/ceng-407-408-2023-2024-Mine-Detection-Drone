import React, { useState } from "react";
import Image from "next/image";
import clsx from "clsx";
import axios from "axios";

import InputText from "./InputText";

import icon from "../assets/icon.png";

interface LoginProps {
  onLogin: () => void;
}

const Login: React.FC<LoginProps> = ({ onLogin }) => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [message, setMessage] = useState("");

  const handleLogin = async () => {
    try {
      const response = await axios.post("/api/login", { username, password });

      if (response.status === 200) {
        setMessage("Success");
        onLogin();
      } else {
        setMessage(response.data.error);
      }
    } catch (error) {
      console.error("Error logging in: ", error);
      setMessage("Invalid");
    }
  };

  return (
    <div className="h-screen flex justify-center items-center">
      <div className="py-12 px-10 shadow-md shadow-rust rounded bg-window">
        <div className="my-4 flex justify-center">
          <Image width="128" height="128" alt="" src={icon} />
        </div>

        <div>
          <InputText
            id="username"
            value={username}
            name="username"
            placeholder="ID"
            onChange={(e) => setUsername(e.target.value)}
          />
          <InputText
            id="password"
            value={password}
            name="password"
            type="password"
            placeholder="Pass"
            onChange={(e) => setPassword(e.target.value)}
          />
        </div>

        <div className="flex justify-center items-center">
          <button
            className="mt-4 py-3 px-8 rounded bg-rust hover:bg-metal font-semibold text-white"
            type="button"
            onClick={handleLogin}
          >
            Login
          </button>
        </div>

        {message && (
          <div
            className={clsx("mt-8 text-center font-medium", {
              "text-valid": message === "Success",
              "text-invalid": message !== "Success",
            })}
          >
            {message}
          </div>
        )}
      </div>
    </div>
  );
};

export default Login;
