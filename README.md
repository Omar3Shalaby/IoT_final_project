# Smart Door Web Application

A responsive web interface for controlling and monitoring a smart door security system.  
It integrates with **Supabase** (for authentication & database) and **HiveMQ MQTT broker** (for real-time communication with the ESP32).

## 🚀 Features
- 🔐 **Authentication**: Sign up / Sign in with Supabase.
- 📊 **Dashboard**: Real-time door status, light level, motion detection, and fire sensor.
- 🎮 **Control Panel**: 
  - Remote lock/unlock
  - Emergency open
  - Change door password
  - Arm/disarm/reset system
- 🚨 **Alerts**: Fire, intrusion, and access notifications.
- 📋 **History**: Complete activity log from Supabase.
- 🌐 **MQTT Communication**: Secure WebSocket connection to HiveMQ Cloud.

## 🛠️ Tech Stack
- **Frontend**: HTML, CSS, JavaScript
- **Auth/DB**: Supabase
- **Messaging**: MQTT.js with HiveMQ Cloud
- **Hosting**: Can be hosted on GitHub Pages, Netlify, or locally

## ⚙️ Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/esp32-web-app.git
   cd esp32-web-app
