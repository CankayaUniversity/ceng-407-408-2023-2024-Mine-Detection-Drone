import { NextApiRequest, NextApiResponse } from "next";
import mysql from "mysql";

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  if (req.method === "POST") {
    const connection = mysql.createConnection({
      host: "localhost",
      user: "root",
      password: "T0N1C_lqsym",
      database: "mine_detection_drone",
    });

    const { username, password } = req.body;

    try {
      await new Promise<void>((resolve, reject) => {
        connection.connect((err) => {
          if (err) reject(err);
          else resolve();
        });
      });

      const results = await new Promise<any[]>((resolve, reject) => {
        connection.query(
          "SELECT * FROM users WHERE username = ? AND password = ?",
          [username, password],
          (err, results) => {
            if (err) reject(err);
            else resolve(results);
          },
        );
      });

      if (results.length === 1) {
        res.status(200).json({ message: "Success" });
        console.log("Success");
      } else {
        res.status(401).json({ error: "Invalid" });
        console.log("Invalid");
      }
    } catch (error) {
      console.error("Error querying database:", error);
      res.status(500).json({ error: "An error occurred while logging in." });
    } finally {
      connection.end();
    }
  } else {
    res.setHeader("Allow", ["POST"]);
    res.status(405).end(`Method ${req.method} Not Allowed`);
  }
}
