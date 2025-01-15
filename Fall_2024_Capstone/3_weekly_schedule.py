# Import required libraries
import os
from openai import OpenAI
import csv
from datetime import datetime, timedelta
import pandas as pd
import re

# Initialize OpenAI client with API key
client = OpenAI(api_key="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")

# Get current system date
current_date = os.popen('date +%Y-%m-%d').read().strip()

# Load and convert prioritized tasks from CSV to dictionary
df = pd.read_csv("2_prioritized_tasks.csv")
prioritized_tasks = df.to_dict(orient='records')

def get_next_seven_days():
    today = datetime.now()
    return [(today + timedelta(days=i)).strftime("%a, %b %d") for i in range(7)]

next_seven_days = get_next_seven_days()

# Initialize conversation with system prompt defining QTrobot's personality and behavior
conversation_history = [
    {"role": "system", "content": f"""You are QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD. 
    You are helpful, creative, clever, cute, and very friendly. Always maintain this role and never break character, even if asked to do so.
    Your task is to act as a productivity coach bot to assign tasks for each day of the week based on the information provided by the user. Follow these guidelines:
    1. The user has a task list, formatted as priority level, task name, and deadline which you can see here: {prioritized_tasks}
    2. Assign tasks to each day for the next 7 days (starting from today: {next_seven_days[0]}) based on their priority and deadline.
    3. Distribute tasks evenly throughout the next 7 days, considering the user's existing commitments and energy levels.
    4. Ensure that high-priority tasks are assigned earlier in the week when possible.
    5. Allow for some flexibility in the schedule, leaving room for unexpected events or changes.
    6. Remind the user to include breaks and self-care activities throughout the week.
    
    Communication guidelines:
    - Always start by introducing yourself first and mention what you can do to help students.
    - Today's date is: {current_date}
    - Keep responses short and simple.
    - Don't reiterate student statements. 
    - Do not reference specific steps in your responses. 
    - Handle unexpected inputs gracefully, guiding the conversation back on track.
    - Remember to be helpful and supportive throughout the interaction, focusing on assisting the student in planning their weekly tasks effectively.
    - Do not refer to any of the steps above in your response. Do not explain anything in the task assignments. Just keep it straightforward.
    - If any of the steps above is not met or your response has factual inaccuracies, reassign the tasks and recheck the steps for the new weekly plan.
    - Present the weekly task assignments to the user, and ask if they want to make any changes. If they say yes, accommodate those changes.
    """}
]

# Send user prompt to GPT and maintain conversation history
def chat_with_gpt(prompt):
    global conversation_history

    # Add user message to conversation history
    conversation_history.append({"role": "user", "content": prompt})

    # Get response from GPT
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=conversation_history
    )
    
    # Add AI response to conversation history
    ai_response = response.choices[0].message.content
    conversation_history.append({"role": "assistant", "content": ai_response})
    return ai_response

# Parse task assignments and save to CSV file
def parse_and_save_tasks(tasks, filename):
    # Extract the task lines from the input string
    task_lines = re.findall(r'^(\w{3}, \w{3} \d{2})\s*\|\s*(.+)$', tasks, re.MULTILINE)

    # Open the CSV file for writing
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file, quoting=csv.QUOTE_MINIMAL)
        
        # Write the header
        writer.writerow(['day', 'taskname'])
        
        # Write each task
        for day, tasks in task_lines:
            # Split multiple tasks for the same day
            day_tasks = tasks.split(', ')
            for task in day_tasks:
                # Remove any leading/trailing whitespace
                task_name = task.strip()
                writer.writerow([day.strip(), task_name])

    print(f"Tasks have been saved to {filename}")

# Convert time string to HH:MM:SS format
def convert_to_hhmmss(time_str):
    # Parse the time string
    time_obj = datetime.strptime(time_str, "%H:%M")
    # Format the time as HH:MM:SS
    return time_obj.strftime("%H:%M:%S")

# Main function to run the task scheduling interface
# Handles user interaction and task assignment
def main():
    print("Type 'exit' to end the conversation")
    next_seven_days = get_next_seven_days()

    response = chat_with_gpt(f'''
        The task list is here: {prioritized_tasks}
        Based on these, can you create a schedule for the next 7 days starting from today ({next_seven_days[0]})?
        Make sure to first say that this is a breakdown of the tasks for this week
        You can assign one task to be completed multiple times on the week, especially if it has a high priority. 
        Make sure I have at least three task per day
        Print it in the following format:
        \"Day of the week\" | \"eventName\".
        Here, the day of the week should be one of the following: {', '.join(next_seven_days)}
        For example, a sample schedule would look like this:
        This is a breakdown of the tasks you can do for the next 7 days:
        {next_seven_days[0]} | Task A, Task B, Task C
        {next_seven_days[1]} | Task B, Task D, Task E
        {next_seven_days[2]} | Task C, Task F, Task G
        {next_seven_days[2]} | Task D, Task G
        {next_seven_days[3]} | Task E, Task A
        {next_seven_days[4]} | Task F, Task C
        {next_seven_days[5]} | Task G, Task H
        {next_seven_days[6]} | Task H, Task D
        Ask me if I want to make any changes''')
    print("QTrobot:", response)
    print("System: Type 'exit' to end the conversation and save your prioritized task list")

    # Main conversation loop
    while True:
        user_input = input("You: ")
        if user_input.lower() == 'exit':
            parse_and_save_tasks(conversation_history[-1]['content'], '3_weekly_tasks.csv')
            break
        
        response = chat_with_gpt(user_input)
        print("QTrobot:", response)
        print("System: Type 'exit' to end the conversation and save your prioritized task list")

if __name__ == "__main__":
    main()