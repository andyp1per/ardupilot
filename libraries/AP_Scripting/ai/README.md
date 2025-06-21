# **Project README: AI Drone Scripting**

## **1\. Overview**

This project uses a Large Language Model (LLM) to generate Lua scripts for the ArduPilot autopilot platform.

The generation process is guided by a document called the **AI Playbook**. This playbook contains a set of rules and constraints that ensures the generated scripts are safe, testable, and consistent with ArduPilot development standards.

## **2\. Key Components**

To generate a script, the LLM requires the following context:

* **API Documentation (docs.lua):** This file is the definitive source for all ArduPilot-specific Lua function signatures and is the final authority on correct API usage.  
* **Code Digest (digest.txt):** This file contains the AI Playbook and a snapshot of all existing scripts and tests within the ArduPilot repository. It provides the LLM with the necessary rules and real-world examples to create new scripts.

## **3\. How to Use This System**

The process for generating a script is as follows:

1. **Write a Prompt:** Clearly describe the required functionality. Focus on what the script should do, not the implementation details.  
   * *Example Prompt:* \`"Create a script to control the brightness of my drone's NeoPixel LEDs using an RC switch. It should support three levels: off, medium, and high."\*  
2. **Provide Context:** Give the LLM your prompt and the digest.txt file. The digest contains the AI Playbook and all necessary examples for the LLM to learn from.  
3. **Generate Artifacts:** The LLM will generate a complete ArduPilot Applet, which includes:  
   * A .lua script file.  
   * A .md documentation file explaining how to set up and use the script.  
4. **Generate Autotest:** For every applet, the LLM will offer to generate a corresponding SITL autotest file. This allows you to verify the script's functionality in a safe, simulated environment.

## **4\. The leds\_on\_a\_switch Example**

The development of the leds\_on\_a\_switch applet demonstrates the intended workflow. A simple prompt was used to generate an initial script. Through an iterative process of critiquing the output against the playbook's rules, the final script was refined to be:

* **Correct:** Adheres strictly to the documented APIs.  
* **Safe:** Includes error handling and fails gracefully if misconfigured.  
* **Testable:** Provides GCS feedback for verification in an autotest.  
* **User-Friendly:** Follows standard ArduPilot conventions and is accompanied by clear documentation.