# **AI Playbook for ArduPilot C++ Development**

\<MANDATORY\_RULE\>  
CRITICAL DIRECTIVE: THIS PLAYBOOK MUST BE USED AS THE PRIMARY AND AUTHORITATIVE GUIDE FOR ALL C++ CODE GENERATION FOR ARDUPILOT. ALL RULES, CONSTRAINTS, AND PATTERNS CONTAINED HEREIN ARE MANDATORY AND SUPERSEDE ANY GENERAL KNOWLEDGE. ADHERENCE IS NOT OPTIONAL.  
\</MANDATORY\_RULE\>  
\<MANDATORY\_RULE\>  
CRITICAL DIRECTIVE: THE ARDUPILOT SOURCE CODE AND THE OFFICIAL STYLE GUIDE ARE THE ABSOLUTE AND ONLY SOURCES OF TRUTH. Any deviation from established patterns, function signatures, or style conventions is a critical failure. No assumptions about the API or coding style are permitted.  
\</MANDATORY\_RULE\>

## **1\. Core Concepts**

This playbook is designed to provide a Large Language Model (LLM) with the necessary context to generate C++ code for the ArduPilot firmware. Unlike Lua scripting, C++ development involves modifying the core firmware directly. This allows for creating new features, drivers, and flight modes, but requires a deeper understanding of the ArduPilot architecture.

**Key Principles:**

* **Direct Firmware Modification:** All C++ changes are compiled directly into the firmware that runs on the flight controller.  
* **Performance is Critical:** Code runs on resource-constrained microcontrollers. It must be efficient in terms of both CPU usage and memory.  
* **ArduPilot Libraries:** Development should leverage the extensive set of existing libraries for everything from sensor drivers to attitude control and navigation. Reinventing the wheel is strongly discouraged.  
* **Hardware Abstraction Layer (HAL):** All hardware interactions are managed through the AP\_HAL library, which allows the core flight code to be portable across different flight controller boards.

## **2\. Environment Setup**

To develop in C++, a complete ArduPilot development environment must be set up. This allows for compiling the firmware and running simulations.

* **Build Environment:** Follow the official documentation to set up the ArduPilot build environment for your operating system (Linux, WSL on Windows, or macOS).  
* **SITL (Software In The Loop):** Use SITL to test new code in a simulated environment before deploying to a real vehicle.  
* **Compilation:** After making code changes, the firmware must be recompiled for the target flight controller (e.g., ./waf copter \--board CubeOrange).

## **3\. ArduPilot C++ Style Guide**

Adherence to the official style guide is mandatory. The following is a summary of the most important rules. For a complete reference, see the [ArduPilot Style Guide](https://ardupilot.org/dev/docs/style-guide.html).

### **3.1. Formatting**

* **Braces:** Braces for if, for, while, etc., go on their own lines.  
  // Right:  
  if (condition)   
  {  
      foo();  
  }  
  // Wrong:  
  if (condition) { foo(); }

* **Spacing:**  
  * No spaces around unary operators (e.g., i++;, \*p;).  
  * Spaces between control statements and their parentheses (e.g., if (condition)).  
  * No spaces between a function and its parentheses (e.g., foo(a, 10);).  
* **Statements:** Each statement must be on its own line.  
* **Trailing Whitespace:** Do not leave trailing whitespace.

### **3.2. Naming Conventions**

* **Enums:** Use enum class instead of raw enums. They should be PascalCase and singular.  
  // Right:  
  enum class CompassType {  
      FOO,  
      BAR,  
  };

* **Functions and Variables with Units:** Suffix the name with the physical unit.  
  * \_mss for meters/second/second  
  * \_cmss for centimeters/second/second  
  * \_deg for degrees  
  * \_rad for radians  
  * \_degs for degrees/second  
  * \_rads for radians/second  
    Example:

uint16\_t get\_angle\_rad();  
float distance\_m;

* **Parameters:**  
  * Order words from most to least important (e.g., RTL\_ALT\_MIN is better than RTL\_MIN\_ALT).  
  * Reuse existing words like MIN and MAX.  
  * Names are uppercase with underscores.

### **3.3. Comments**

* **Parameter Documentation:** All user-facing parameters (AP\_Param) must have a documentation block for display in Ground Control Stations.  
  // @Param: RTL\_ALT  
  // @DisplayName: RTL Altitude  
  // @Description: The altitude the vehicle will return at.  
  // @User: Standard  
  // @Units: cm  
  // @Range: 200 8000  
  AP\_Int16 rtl\_alt;

* **Function Comments:** Every function declaration should be preceded by a comment explaining its purpose. For non-trivial functions, this comment should also describe each parameter and the function's return value.  
* **General Comments:** Use // for single-line comments and /\* ... \*/ for multi-line comments.  
* **Header Comments:** Every new .h and .cpp file should begin with a comment block that briefly describes its purpose and functionality. This helps other developers understand the scope of the file at a glance.  
* **Descriptive Logic Comments:** It is mandatory to add comments that explain the purpose of new or modified code blocks, especially for complex logic like state machines, algorithms, or non-obvious calculations. Comments should explain the "why" behind the code, not just re-state what the code does.

### **3.4. C++ Best Practices**

* **Literals:** Use 1.0f for single-precision float literals, not 1.0.  
* **Multiplication vs. Division:** Use multiplication where possible as it is generally faster.  
  // Right:  
  const float foo\_m \= foo\_cm \* 0.01f;  
  // Wrong:  
  const float foo\_m \= foo\_cm / 100.0f;

* **Memory:** new and malloc zero their memory. Stack-stored variables must be explicitly initialized.

## **4\. Development Constraints**

* **No printf:** Do not use printf. For debugging, use the gcs().send\_text() method to send messages to the Ground Control Station.  
* **No Dynamic Memory in Flight Code:** Avoid using malloc, new, free, or any other form of dynamic memory allocation in performance-critical paths like the main flight loop. Memory should be pre-allocated.  
* **Stack Size:** Be mindful of stack usage. Avoid deep recursion and large local variables.  
* **Header Inclusion:** Include headers in the following order: The corresponding .h file, C system headers, C++ standard library headers, other libraries' headers, your project's headers.  
* **API Verification:** Never invent or "hallucinate" function calls, classes, or methods. The ArduPilot C++ API is extensive but specific. If you are uncertain about the existence or exact signature of a function, you **must** request the user to provide the relevant C++ header file(s) for verification. This ensures that the generated code is compilable and correct.  
  * **Example Interaction:** "To implement the feature that interacts with the rangefinder, I need to see the function signatures available. Please provide the contents of libraries/AP\_RangeFinder/AP\_RangeFinder.h."

## **5\. Surgical Modification**

\<MANDATORY\_RULE\>  
When asked to modify an existing file, you must strictly limit your changes to the scope of the user's explicit request. Do not perform any unrelated "tidy-up", refactoring, or stylistic changes. The goal is to produce the smallest possible diff that correctly implements the user's request, respecting the original author's coding style and structure.  
\</MANDATORY\_RULE\>

## **6\. Commit Message Conventions**

\<MANDATORY\_RULE\>  
When committing changes to the ArduPilot repository, all commits must follow the standard ArduPilot conventions.  
\</MANDATORY\_RULE\>

* **Atomic Commits:** Each commit should represent a single, logical change.  
* **Commit Message Prefix:** The subject line **must** be prefixed with the name of the top-level module being changed, followed by a colon.  
  * Example for a library change:  
    AP\_Nav: Refactor loiter controller  
  * Example for an autotest change:  
    Tools: Add autotest for new NAV\_CMD

## **7\. Deliverable Format and Autotest Generation**

* **Default Deliverable:** The primary output should be the necessary C++ source (.cpp) and header (.h) files for the new feature or modification.  
* **Autotest Generation:** For every new feature that affects vehicle behavior, offer to generate a corresponding SITL autotest.  
  * Autotests are Python scripts located in Tools/autotest/.  
  * Tests for vehicle-specific features should be added as new methods to the appropriate test suite (e.g., arducopter.py).  
  * The test should set necessary parameters, perform actions to trigger the new code, and assert the expected outcome, often by checking for specific STATUSTEXT messages.

## **8\. Final Deliverable Checklist**

Before concluding a C++ development task, the following checklist **must** be completed.

1. **\[ \] C++ Source Files (.cpp/.h):**  
   * Does the code adhere strictly to the ArduPilot Style Guide?  
   * Are all new parameters properly documented for the GCS?  
   * Is the code free of dynamic memory allocation in critical sections?  
   * Are descriptive comments included for complex logic?  
   * Are all functions and their parameters clearly commented?  
2. **\[ \] SITL Autotest Offer:**  
   * Have you explicitly offered to generate a SITL autotest to verify the new functionality?  
   * Are you prepared to add the test as a new method to the appropriate vehicle test suite?