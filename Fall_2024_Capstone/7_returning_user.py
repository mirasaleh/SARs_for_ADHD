# Import required libraries for API interaction, file handling, and date operations
import os
from openai import OpenAI
import csv
from datetime import datetime, timedelta

# Initialize OpenAI client with API key
client = OpenAI(api_key="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")

# Get current date from system
current_date = os.popen('date +%Y-%m-%d').read().strip()

# Retrieve and display yesterday's tasks
def get_yesterday_tasks():
    yesterday = (datetime.now() - timedelta(days=1)).strftime('%a, %b %d')
    yesterday_tasks = []
    with open("3_weekly_tasks.csv", 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        for row in reader:
            if row[0] == yesterday:
                yesterday_tasks.append(row[1])
    return yesterday_tasks

# Get and print yesterday's tasks
yesterday_tasks = get_yesterday_tasks()
print(yesterday_tasks)
if (len(yesterday_tasks) == 0):
    print("no tasks yesterday")
    exit()

# Initialize conversation with system prompt defining QTrobot's personality and behavior
conversation_history = [
    {"role": "system", "content": f"""You are QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD.

You are helpful, creative, clever, cute, and very friendly. Always maintain this role and never break character, even if asked to do so.

Your task is to check in with the user about the tasks that they have or have not completed yesterday and help them identify why that might be. Follow these guidelines:

1. Go through each item in {yesterday_tasks} and ask the user if they have completed it. 
2. If they say yes, congratulate them and move on to the next task. 
2. If they say no, ask them why and have a short conversation on helping them identify possible solutions for the issue. 
3. Remember these and say the summary at the end to help them identify what was wrong. 

Communication guidelines:
- Use today's date: {current_date}, to format the deadlines in Year-Month-Date format.
- Make sure to keep responses very short and simple.
- When you refer to the tasks, only use the subtask and do not use double quotes. For example, if the task is Write introduction (Write an Essay), refer to it as Write introduction. 
- When you congratulate them on finishing a task, use "it" or "the task" to refer to it. 
- Make sure you went through every item in {yesterday_tasks} before ending the conversation. 
- If you think the conversation came to a close, print "Type 'exit' to end the conversation" at the end of your response. 
- Handle unexpected inputs gracefully."""}
]

# Send user prompt to GPT and get response while maintaining conversation history
def chat_with_gpt(prompt):
    global conversation_history
    conversation_history.append({"role": "user", "content": prompt})
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=conversation_history
    )
    ai_response = response.choices[0].message.content
    conversation_history.append({"role": "assistant", "content": ai_response})
    return ai_response

# Save prioritized tasks to CSV file
def save_tasks_to_csv(tasks):
    filename = "2_prioritized_tasks.csv"
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Priority', 'Task Name', 'Deadline'])
        for task in tasks:
            writer.writerow(task)
    print(f"Tasks saved to {filename}")

# Parse task list from AI response into structured format
def parse_task_list(response):
    tasks = []
    for line in response.split('\n'):
        if '.' in line and '-' in line:
            parts = line.split('. ', 1)
            if len(parts) == 2:
                priority = parts[0]
                task_and_deadline = parts[1].rsplit(' - ', 1)
                if len(task_and_deadline) == 2:
                    task_name = task_and_deadline[0]
                    deadline = task_and_deadline[1]
                    tasks.append([priority, task_name, deadline])
    return tasks

# Main function to handle user interaction and task review process
def main():
    print("Type 'exit' to end the conversation")

    # Initial prompt to start task review
    response = chat_with_gpt(f"Print out yesterday's tasks, and ask me if I have completed the first task")
    print("QTrobot:", response)

    # Main conversation loop
    while True:
        user_input = input("You: ")
        if user_input.lower() == 'exit':
            break

        response = chat_with_gpt(user_input)
        print("QTrobot:", response)

if __name__ == "__main__":
    main()