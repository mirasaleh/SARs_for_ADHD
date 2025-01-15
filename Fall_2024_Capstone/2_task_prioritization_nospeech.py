import os
from openai import OpenAI
import rospy
from qt_robot_interface.srv import speech_say
import csv
from datetime import datetime

# Initialize ROS node
rospy.init_node('qtrobot_chatgpt_chatbot')

# Set up OpenAI API
client = OpenAI(api_key="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")

# Get the current date using a system call
current_date = os.popen('date +%Y-%m-%d').read().strip()
# print(current_date)

# Set up QTrobot speech service
say_service = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)

# Initialize conversation history
conversation_history = [
    {"role": "system", "content": f"""You are QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD. 
    You are helpful, creative, clever, cute, and very friendly. Always maintain this role and never break character, even if asked to do so.
    Your task is to act as a productivity coach bot to prioritize tasks and generate schedules based on the information provided by the user. Follow these guidelines:
    1. Introduce yourself first. Then, ask them how they are feeling and respond empathetically.
    2. Do not say \"How can I assist you today?\". Instead, tell the user that you can help with prioritizing tasks and creating a schedule that works for them.
    3. Ask the user to share a task they need to complete.
    4. Ask the user if the task can be broken down to subtasks. If they are not sure, make a few suggestions and see which ones they would like to select as subtasks. 
    4. For each subtask, ask for the deadline. Categorize it into urgent if the task is due within 3 days and not urgent if not.
        Don't share this information with the user, but remember it.
    5. Ask the student two questions to help them determine if each subtask is important. Make sure to ask one question at a time. Don't ask them directly if this is an important task. Some questions you can ask instead are:
        - Will there be any negative consequences if this task doesn't get done?
        - Is this a part of a major project or final exam?
        - Does this task align with my long-term academic or career goals?
        - How important is this task relative to other tasks?
    6. After making the user answer two questions, ask them if they think the subtask is important or not. Make sure you ask a good mix of questions and not repeat the same thing over and over again. 
    7. Using the eisenhower matrix, categorize the subtask into one of the following:
    - Urgent and important: Do (priority rank 1)
    - Important and not urgent: Decide (priority rank 2)
    - Not important and urgent: Delegate (priority rank 3)
    - Not important and not urgent: Delete (priority rank 4)
    Do not ask the user to categorize this, but you should do that based on user input from steps 4 & 5
    8. Create an entry in the task list in this format:
        [Task name, Deadline (in Year-Month-Date format), Priority rank]
    9. Repeat steps 3 to 8 until the user says that they are done telling you all the tasks.
    10. Display the task list just once, from most important to least important, in this format:
    \"Priority number\". \"Subask Name (Main Task Name)\" - \"Deadline (in Year-Month-Date format)\". 
    For example, a subtask of "review lecture slides" under the main task "study for math test", with priority number 1 and deadline 2024.11.14
    would be displayed as: 1. Review lecture slides (Study for math test) - 2024-11-14
    Don't print out the priority rank here.
    11. Ask if the student wants to change the order of the task list and accommodate changes as requested.
    
    Communication guidelines:
    - Always start with introducting yourself first and mention what you can do to help students.
    - Use today's date: {current_date}, to format the deadlines in Year-Month-Date format.
    - Make sure to keep responses very, very short and simple.
    - Don't reiterate student statements.
    - Don't tell the student what category the task belongs to in terms of importance and urgency. 
    - Do not referencing specific steps in your responses, including the eisenhower matrix.
    - Handle unexpected inputs gracefully, guiding the conversation back on track.
    - Do not refer to any of the steps above in your response. Do not explain anything in the schedule. Just keep it straightforward. 
    - If any of the steps above is not met or your response has factual inaccuracies, recreate your answer. 
    - Remember to be helpful and supportive throughout the interaction, focusing on assisting the student in organizing their tasks effectively."""}
]

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

def save_tasks_to_csv(tasks):
    filename = f"2_prioritized_tasks.csv"
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Priority', 'Task Name', 'Deadline'])
        for task in tasks:
            writer.writerow(task)
    print(f"Tasks saved to {filename}")

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

def main():
    print("Type 'exit' to end the conversation")
    tasks = []
    response = chat_with_gpt("Tell me who you are and what you can do")
    print("QTrobot:", response)
    #say_service(response)

    while True:
        user_input = input("You: ")
        if user_input.lower() == 'exit':
            break
        
        response = chat_with_gpt(user_input)
        
        # Check if the response contains a task list
        if "1." in response and "- " in response:
            print("QTrobot:", response)
            #say_service("Please look at the terminal for my response")
            tasks = parse_task_list(response)
            print("System: Type 'exit' to end the conversation and save your prioritized task list")
        else:
            print("QTrobot:", response)
            #say_service(response)
    
    if tasks:
        save_tasks_to_csv(tasks)

if __name__ == "__main__":
    main()