# 🚦 STM32 FreeRTOS Traffic Light with Emergency Mode 

A traffic light system implemented on STM32 using FreeRTOS, with emergency mode triggered by an external button press. This project simulates a pedestrian crossing system where the button overrides normal operation to immediately activate the green LED for 5 seconds, allowing pedestrians to cross safely.

---

## 🛠️ Project Overview
This project demonstrates:
- **Real-time task management** using FreeRTOS.  
- **Interrupt handling** for emergency mode activation.  
- **Task synchronization** with mutexes to avoid race conditions.  
- **LED control** to simulate traffic lights (Red, Green, Blue, and Orange).  

---

## 🎯 Key Features
- **Standard Traffic Light Sequence**:   
  - Red -> Orange -> Green  
- **Emergency Mode**:  
  - Button press interrupts normal flow to activate green LED and blink blue LED for 5 seconds.  
- **LED Management**:  
  - Blue LED blinks during emergency mode.  
  - Red LED resumes after emergency ends.  

---

## 🔧 Components and Tools
- **STM32F4 Discovery Board**  
- **FreeRTOS** - Real-time operating system for task management.  
- **STM32CubeIDE** - Development environment for STM32.  
- **HAL (Hardware Abstraction Layer)** - GPIO and system clock configurations.  

---

## 📂 Project Structure
```bash
📁 STM32-Traffic-Light
├── Inc/
│   ├── main.h
│   └── FreeRTOSConfig.h
├── Src/
│   ├── main.c
│   ├── freertos.c
│   └── stm32f4xx_it.c
├── Middlewares/
└── README.md
```
## 🚦 How It Works

### Green Task:
- Activates the green LED for 5 seconds, followed by orange for 2 seconds.

### Red Task:
- Activates the red LED for 5 seconds.

### Emergency Task:
- On button press, overrides normal flow to turn on the green LED and blink blue.

### Blue LED:
- Blinks at 500ms intervals during emergency mode.

## 🔑 Code Breakdown

### Task Creation
Tasks for red, green, and emergency modes are created during initialization:
```c
xTaskCreate(GreenTask, "GreenTask", TASK_STACK_SIZE, NULL, 1, &greenTaskHandle);
xTaskCreate(RedTask, "RedTask", TASK_STACK_SIZE, NULL, 2, &redTaskHandle);
xTaskCreate(BlueTask, "BlueTask", TASK_STACK_SIZE, NULL, 3, &blueTaskHandle);
xTaskCreate(urgentGreen, "urgentGreen", TASK_STACK_SIZE, NULL, 3, &urgentGreenHandle);
```
### Interrupt Handler
The EXTI0 interrupt (for the button) triggers emergency mode:
```c
void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskResumeFromISR(urgentGreenHandle);
    xTaskResumeFromISR(blueTaskHandle);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```
### Mutex Protection
A mutex protects the emergencyFlag to avoid race conditions:
```c
Copy code
if(xSemaphoreTake(xEmergencyFlagMutex, portMAX_DELAY) == pdTRUE) {
    emergencyFlag = 0;
    xSemaphoreGive(xEmergencyFlagMutex);
}
```
## ⚙️ Setup and Build
### Clone the repository:
```bash
git clone https://github.com/username/STM32-Traffic-Light.git
```
### Open the project in STM32CubeIDE.

### Build and flash the project to your STM32F4 Discovery board.

## 📸 Demo
You can visit my linkedin profile to see the demo :
```bash
https://www.linkedin.com/feed/update/urn:li:activity:7277312173813514240/
```

## 🧩 Why FreeRTOS?

- **Task Priority**: Ensures critical tasks (like emergency) run immediately.
- **Efficient Scheduling**: No delays in regular operations.
- **Resource Management**: Mutex prevents shared resource conflicts.

## 📜 License
MIT License - feel free to use and modify the code.

## 🤝 Contributing
Pull requests and suggestions are welcome!

## 📧 Contact
If you have any questions or ideas, feel free to reach out!

