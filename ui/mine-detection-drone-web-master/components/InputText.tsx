import React, { useState, useEffect } from "react";
import clsx from "clsx";

interface InputTextProps {
  id: string;
  value: string;
  name: string;
  type?: string;
  label?: string;
  placeholder?: string;
  onChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

function isEmpty(value: string): boolean {
  return value.trim() === "";
}

function isValid(value: string): boolean {
  return isEmpty(value) || value.length >= 16;
}

const InputText: React.FC<InputTextProps> = ({
  id,
  value,
  name,
  type,
  label,
  placeholder,
  onChange,
}) => {
  const [wasInvalid, setWasInvalid] = useState(false);
  const [animate, setAnimate] = useState(false);

  useEffect(() => {
    if (wasInvalid && isValid(value) && !isEmpty(value)) {
      setWasInvalid(false);
      setAnimate(true);
      setTimeout(() => setAnimate(false), 500);
    } else if (!wasInvalid && !isValid(value) && !isEmpty(value)) {
      setWasInvalid(true);
    } else if (wasInvalid && isEmpty(value)) {
      setWasInvalid(false);
    }
  }, [value, wasInvalid]);

  return (
    <div>
      <label htmlFor={id} className={"font-semibold text-default"}>
        {label}
      </label>

      <input
        className={clsx(
          "min-w-[256px] my-2 p-3 border-2 rounded focus:outline-none text-input",
          wasInvalid && "border-invalid",
          animate && "pulse-green",
        )}
        id={id}
        value={value}
        name={name}
        type={type}
        placeholder={placeholder}
        onChange={onChange}
      />

      {wasInvalid ? (
        <p className={clsx("text-sm", { "text-invalid": !isValid(value) })}>
          {!isValid(value) ? "Length must be at least 16 characters." : ""}
        </p>
      ) : (
        <p className="text-sm" style={{ visibility: "hidden" }}>
          Placeholder
        </p>
      )}
    </div>
  );
};

export default InputText;
