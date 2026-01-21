

# 🚗 Voice Automated Car with Obstacle Detection

A **voice-controlled smart robotic car** built using **Arduino Uno** that supports **wireless navigation** and **real-time obstacle detection**.
This project demonstrates the practical integration of **embedded systems, IoT, automation, and intelligent navigation** in a **low-cost, educational prototype** 

---

## 📌 Project Overview

The **Voice Automated Car with Obstacle Detection** is a smart mobility prototype that allows users to control a robotic car using **voice commands** via a smartphone.
An **ultrasonic sensor** continuously monitors the surroundings and automatically stops or redirects the car when obstacles are detected, ensuring **safe navigation**.

This project was developed as part of the course:

> **Practical Robotics Projects with Arduino (CSE 4571)**
> Department of Computer Science & Engineering
> Siksha ‘O’ Anusandhan (Deemed to be University)

---

## ✨ Key Features

* 🎙️ **Voice-Based Control** (Hands-free operation)
* 📡 **Wireless Communication** using ESP8266 Wi-Fi Module
* 🚧 **Real-Time Obstacle Detection** with Ultrasonic Sensor
* ⚙️ **Smooth Motor Control** (Reduced jerks & stable movement)
* 🔧 **Low-Cost & Modular Design**
* 🎓 **Educational & Beginner-Friendly**

---

## 🧠 System Architecture

**Input → Processing → Action**

* **Input:** Voice commands from smartphone
* **Processing:** Arduino Uno interprets commands & sensor data
* **Action:** Motor driver controls DC motors with safety overrides

---

## 🛠️ Hardware Components

| Component                 | Description                     |
| ------------------------- | ------------------------------- |
| Arduino Uno               | Central microcontroller         |
| ESP8266-01                | Wi-Fi module for voice commands |
| Ultrasonic Sensor         | Obstacle detection              |
| L298N Motor Driver        | Motor speed & direction control |
| DC Motors + Wheels        | Vehicle movement                |
| Servo Motor               | Sensor rotation                 |
| Li-ion Battery Pack       | Power supply                    |
| Jumper Wires & Breadboard | Circuit connections             |

---

## 💻 Software & Tools Used

* **Arduino IDE**
* **Embedded C / C++**
* **Libraries Used:**

  * `ESP8266WiFi.h`
  * `Servo.h`
* **Serial Monitor** (Debugging)

---

## 🗣️ Supported Voice Commands

| Command  | Action         |
| -------- | -------------- |
| Forward  | Move forward   |
| Backward | Move backward  |
| Left     | Turn left      |
| Right    | Turn right     |
| Stop     | Immediate halt |

> ⚠️ If an obstacle is detected within the safety range, **movement commands are overridden automatically**.

---

## 🔬 Testing & Performance

* ✅ **Command Accuracy:** ~96%
* ⏱️ **Response Time:** ~250 ms
* 🛑 **Collision Incidents:** 0
* 🔁 **Reliability:** ~94% over 100 trials

The system showed **consistent performance**, smooth motion, and stable wireless connectivity during repeated testing 

---

## 💰 Cost Analysis

| Item               | Cost (₹)   |
| ------------------ | ---------- |
| Total Project Cost | **₹2,255** |

> Significantly cheaper than commercial robotic platforms, making it ideal for **students and academic labs**.

---

## 🚀 Applications

* Robotics & Embedded Systems Labs
* Educational Demonstrations
* Smart Mobility Prototypes
* IoT & Automation Learning Projects

---

## ⚠️ Limitations

* Limited Wi-Fi range (local network dependent)
* Sensor accuracy affected by environmental noise
* Basic predefined command set (no AI decision-making)

---

## 🔮 Future Enhancements

* 📱 Dedicated Mobile App
* 🧠 AI-Based Navigation
* 🎙️ Advanced Speech Recognition
* 📷 Camera-Based Object Detection
* 🔋 Improved Power Management

---

## 👥 Team Members

* **Rahul Mallik**
* **Utkarsh Ayush**
* **Chiranjeev Rout**
* **Samparna Mangaraj**

---

## 📄 Project Report

A detailed **end-term project report** including design, testing, cost analysis, and documentation is available in this repository.

---

## 📜 License

This project is developed **for academic and educational purposes**.
You are free to **use, modify, and learn** from it with proper credit.

---

## ⭐ If you like this project

Don’t forget to **star ⭐ the repository** and share it with fellow robotics enthusiasts!

---

If you want, I can also:

* Add **GitHub badges**
* Create a **project diagram**
* Write a **short project description for resumes**
* Optimize it for **GitHub portfolio / placements**

Just tell me 👍
