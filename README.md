# Leveraging Social Robots to Address Productivity Challenges in College Students with ADHD
Attention-Deficit Hyperactivity Disorder (ADHD) significantly impacts the academic, social, emotional, and psychological well-being of young adults. While various treatments and medications are available, there remains a notable absence of practical tools tailored to help them navigate their daily challenges. To bridge this gap, our research centers on designing software solutions that integrate seamlessly with the QTrobot, developed by LuxAI. 

Read the entire report [here](https://drive.google.com/file/d/1NR1xArXdTSeb3dZjmCvjjnME3_LJePbJ/view?usp=sharing). 


## Modules and Features
- **Conversation**: Facilitates interactive and supportive dialogue through the use of GPT-4o, acting as a virtual productivity coach to assist users in managing tasks and maintaining focus.
- **Task Prioritization and Schedule Generation**: Implements advanced task management, including task decomposition, priority setting, and weekly/daily schedule creation using tools like the Eisenhower Matrix. 
- **Pomodoro Session**: Provides structured work sessions with timed intervals, break, and website-blocking functionality to minimize distractions.
- **Engagement Detection**: Continuously monitors the user's attention levels using a deep learning framework to assess and respond to engagement in real time.
- **Emotion Recognition**: Detects and analyzes the user's emotional state using the DeepFace library, enabling empathetic interactions and tailored responses.

## Instructions to Run

### Prerequisites

Before running the code, ensure the following software and libraries are installed on your system:

1. **DeepFace**: Required for emotion detection.
2. **Engagement Detection Model**: Used to analyze engagement levels.
3. **openai**: Enables interaction with OpenAI's GPT models for natural language processing and conversation generation
4. **notion-client**: Provides an interface for interacting with Notion's API to manage tasks and create to-do lists
5. **google-auth-oauthlib**: Handles OAuth 2.0 authentication flow for Google Calendar integration
6. **google-auth**: Manages Google API authentication credentials and tokens
7. **googleapiclient**: Provides access to Google Calendar API for event management and scheduling
8. **pandas**: Handles data manipulation and analysis for task management and scheduling
9. **numpy**: Supports numerical operations and data processing for various computations
10. **requests**: Manages HTTP requests for API interactions and data transfer
11. **httpx (version 0.15.0 or higher)**: Modern HTTP client with async support for making API requests

### API Keys and Credentials
- OpenAI API key
- Notion API integration token/access token
- Google Calendar API credentials

## Setup Instructions

### Step 1: Install Required Libraries

Run the following commands in your terminal to install the necessary Python libraries:

```bash
pip install openai notion-client httpx pandas google-auth-oauthlib
pip install openai[embeddings]
pip install openai[datalib]
pip install openai[wandb]
pip install deepface
```
For more information on OpenAI, visit https://openai.com/  
For more information on Notion, visit https://www.notion.so/  
For more information on pandas, visit https://pandas.pydata.org/  
For more information on HTTPX, visit https://www.python-httpx.org/  
For more information on Google Auth OAuthlib, visit https://google-auth-oauthlib.readthedocs.io/  
For more information on DeepFace, visit the [DeepFace documentation](https://pypi.org/project/deepface/).

### Step 3: Download the Engagement Detection Model

Clone the engagement detection repository from GitHub to access the required model files. Run the following command in your terminal:

```bash
git clone https://github.com/LCAS/engagement_detector.git
```

For additional details, refer to the [repository documentation](https://github.com/LCAS/engagement_detector?tab=readme-ov-file).

### Step 4: Set Up API Keys and Credentials
Ensure you have the necessary API keys and credentials:
- OpenAI API key
- Notion API integration token/access token
- Google Calendar API credentials  
Place these in the appropriate configuration files or environment variables as required by the scripts.

### Step 5: Set Up the Server

The server script (`server.py`) manages communication between different modules. To start the server, run the following command in a terminal:

```bash
python3 server.py
```

### Step 6: Run the Main Menu Script

In a separate terminal, navigate to the project directory and run the main menu script (`0_main_menu.py`) to begin interacting with the system:

```bash
python3 0_main_menu.py
```

## Additional Notes

- Make sure to configure any additional dependencies or environment variables as required by your system.
- To start a new user session, run ```make clean``` to delete all csv files and the token.json file
- When using the Google Calendar API, you may need to set up OAuth 2.0 credentials and handle the authentication flow as shown in the ```6_export_events.py``` script.
- The Notion API integration requires proper setup and permissions as demonstrated in the ```4_notion_push.py``` script.
