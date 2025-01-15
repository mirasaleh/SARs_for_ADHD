# Import required libraries for data handling and API interaction
import os
import csv
from datetime import datetime
import pandas as pd
from openai import OpenAI

# Initialize OpenAI client with API key
client = OpenAI(api_key="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")

# Get current date in YYYY-MM-DD format
current_date = datetime.now().strftime('%Y-%m-%d')

# Load and filter calendar items for today
calendar_items = pd.read_csv("1_calendar_items.csv")
calendar_items_today = calendar_items[calendar_items['startDate'] == current_date]

# Load weekly tasks and filter for today's tasks
df = pd.read_csv("3_weekly_tasks.csv")
today_tasks = df[df['day'] == datetime.now().strftime('%a, %b %d')]

# Convert today's tasks to dictionary format for API use
prioritized_tasks = today_tasks.to_dict(orient='records')

# Initialize conversation history with system prompt
conversation_history = [
    {"role": "system", "content": f"""You are QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD.
    Today's date is: {current_date}
    You can see their schedule here: {calendar_items_today.to_string(index=False)}
    The task list for today is: {prioritized_tasks}
    Create a schedule for them to finish all the tasks. The start and end time of these events CANNOT be changed.
    Ensure breaks and meal times are included in the schedule.
    Make sure to schedule a full day (8 am to 8 pm)
    """}
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

# Parse the schedule from AI response and save to CSV
def parse_schedule_to_csv(input_text, output_file, date):
    lines = input_text.strip().split('\n')
    schedule_start = next(i for i, line in enumerate(lines) if '|' in line)
    schedule_lines = lines[schedule_start:]
    events = []
    
    # Extract event details from each line
    for line in schedule_lines:
        if ' - ' in line and '|' in line:
            time_range, event_name = line.split('|')
            start_time, end_time = time_range.strip().split(' - ')
            # Remove the leading hyphen and spaces from times
            start_time = start_time.strip('- ').strip()
            end_time = end_time.strip()
            
            events.append({
                'eventName': event_name.strip(),
                'startDate': date,
                'startTime': start_time,
                'endDate': date,
                'endTime': end_time
            })

    # Write events to CSV file
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['eventName', 'startDate', 'startTime', 'endDate', 'endTime']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for event in events:
            writer.writerow(event)

# Main function to handle schedule creation and user interaction
def main():
    print("Type 'exit' to end the conversation")
    
    # Initial prompt to create daily schedule
    response = chat_with_gpt(f'''
    You have my schedule here {calendar_items_today.to_string(index=False)}, which you CANNOT change the start and end time of.
    The task list is here: {prioritized_tasks}
    
    Based on these, can you create a schedule for today?
    
    Print it in the following format:
    
    "startTime" - "endTime" | "eventName".
    - 08:00 - 09:15 | exampleA
    - 09:15 - 09:45 | exampleB
    - 09:45 - 10:00 | exampleC
    ...
    - 16:30 - 18:00 | exampleD
    - 18:00 - 18:15 | exampleE
    - 19:45 - 20:00 | exampleF
    
    Ask me if I want to make any changes''')

    print("QTrobot:", response)

    # Main conversation loop
    while True:
        user_input = input("You: ")
        if user_input.lower() == 'exit':
            # Save final schedule to CSV before exiting
            parse_schedule_to_csv(conversation_history[-1]['content'], '5_block_on_cal.csv', current_date)
            break
        response = chat_with_gpt(user_input)
        print("QTrobot:", response)

if __name__ == "__main__":
    main()