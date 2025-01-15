import subprocess
import csv
import getpass
import sys
import os
from datetime import datetime, timedelta

import rospy
import threading
import time

from engagement_main import MainApp
from emotion_main import EmotionMonitor

# Print a visual divider line for better UI readability
def print_divider():
    print("==========================================")

# Display welcome message and introduce QTrobot's capabilities
def print_intro():
    print("Welcome! I am QTrobot, a humanoid social robot assistant designed to support college students and young adults with ADHD.")
    print("I am a productivity coach bot that can help you prioritize tasks and generate a schedule.")

# Handle user login by validating credentials against stored user data
# Returns: True if login successful, False otherwise
def login():
    print("=====================login=====================")
    username = input("Enter your username: ")
    password = getpass.getpass("Enter your password: ")

    with open("0_userinfo.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if len(row) >= 2 and row[0] == username and row[1] == password:
                print("Login successful!")
                return True
    
    print("Invalid username or password. Please try again.")
    return False

# Handle new user registration with input validation
def signup():
    print("=====================Sign Up=====================")
    while True:
        username = input("Enter a username (4-25 characters): ")
        if 4 <= len(username) <= 25:
            if not username_exists(username):
                break
            else:
                print("Error: Username already exists. Please choose a different username.")
        else:
            print("Error: Username must be between 4 and 25 characters. Please try again.")

    # Password validation loop
    while True:
        password = getpass.getpass("Enter a password (4-25 characters): ")
        if 4 <= len(password) <= 25:
            break
        else:
            print("Error: Password must be between 4 and 25 characters. Please try again.")

    # Save new user credentials
    with open("0_userinfo.csv", "a", newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow([username, password])
    
    print("Sign up successful! Log in with the registered information.")

# Check if username already exists in the system
def username_exists(username):
    with open("0_userinfo.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row[0] == username:
                return True
    return False

# Retrieve tasks from previous day
def get_yesterday_tasks():
    yesterday = (datetime.now() - timedelta(days=1)).strftime('%a, %b %d')
    yesterday_tasks = []
    with open("3_weekly_tasks.csv", 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        for row in reader:
            if row[0] == yesterday:
                yesterday_tasks.append(row[1])
                print(row)
    return yesterday_tasks

# Determine user's progress in the system and provide appropriate welcome message
def check_returning_user():
    # List of files that indicate user progress
    files_to_check = [
        "1_calendar_items.csv",
        "2_prioritized_tasks.csv",
        "3_weekly_tasks.csv"
    ]
    
    completed_steps = 0
    
    # Check which steps user has completed
    for i, file in enumerate(files_to_check, start=1):
        if os.path.exists(file):
            completed_steps = i
        else:
            break

    if os.path.exists("5_block_on_cal.csv"):
        completed_steps = 5
    
    # Display appropriate welcome message based on user progress
    if completed_steps == 0:
        print("Welcome, new user! Let's get started with importing your schedule from Google Calendar.")
    elif completed_steps == 1:
        print("Welcome back! Let's prioritize your tasks.")
    elif completed_steps == 2:
        print("Welcome back! Let's create a weekly task list.")
    elif completed_steps == 3:
        print("Welcome back! Let's export your weekly task list to notion or create a daily schedule.")
    elif completed_steps == 5:
        if (len(get_yesterday_tasks()) == 0):
            print("Welcome! It looks like you are a new user or have not used the program in the last 7 days, let's get started with importing your schedule from Google Calendar.")
        else:
            print("Welcome back! Tell me how your study session went yesterday.")
            subprocess.run(['python', '7_returning_user.py'])

    print_divider()

# Display main menu options
def print_menu():
    print("Please enter the number to select the task that you want to execute:")
    print("1. Import your current schedule from Google Calendar")
    print("2. Converse with me to prioritize tasks")
    print("3. Converse with me to assign tasks for the next 7 days")
    print("4. Export the weekly task breakdown to Notion")
    print("5. Converse with me to schedule the tasks assigned to today")
    print("6. Export the generated daily schedule to Google Calendar")
    print("7. Start a focus session")
    print("Enter 'exit' to quit")

# Execute user's menu choice with appropriate validation
# Execute user's menu choice with appropriate validation
def execute_menu(choice):
    if choice == '1':
        subprocess.run(['python', '1_fetch_events.py'])
    elif choice == '2':
        subprocess.run(['python', '2_task_prioritization.py'])
    elif choice == '3':
        if os.path.exists("2_prioritized_tasks.csv"):
            subprocess.run(['python', '3_weekly_schedule.py'])
        else:
            print("You must complete step 2 before this step.")
    elif choice == '4':
        if os.path.exists("3_weekly_tasks.csv"):
            subprocess.run(['python', '4_notion_push.py'])
        else:
            print("You must complete step 3 before this step.")
    elif choice == '5':
        if os.path.exists("3_weekly_tasks.csv"):
            subprocess.run(['python', '5_create_schedule.py'])
        else:
            print("You must complete step 3 before this step.")
    elif choice == '6':
        if os.path.exists("5_block_on_cal.csv"):
            subprocess.run(['python', '6_export_events.py'])
            print("Would you like to start a focus session?")
        else:
            print("You must complete step 5 before this step.")
            
    elif choice == '7':
        print("Please wait until I get some things together. This will only take a few moments. Thanks!")
        if os.path.exists("2_prioritized_tasks.csv"):
            import sys
            import re

            # Define the TerminalFilter class inline
            class TerminalFilter:
                """
                A custom stdout filter that prints lines containing "Alex:" and also prints 
                the prompt "User:" on the same line (no newline). 
                Additionally, it allows specific ROS log messages to pass through without 
                the [INFO] and [TIMESTAMP] prefixes.
                """
                def __init__(self, original_stdout):
                    self._original_stdout = original_stdout
                    # Pattern to match Alex's responses
                    self.keep_pattern_alex = re.compile(r'(Alex:)', re.IGNORECASE)
                    # Pattern to match the exact "User:" prompt
                    self.keep_pattern_user = re.compile(r'^(User:)$', re.IGNORECASE)
                    # List of allowed ROS log messages
                    self.allowed_log_messages = [
                        "Starting short break...",
                        "Stopping Pomodoro session...",
                        "Resuming Pomodoro timer...",
                        "Pausing Pomodoro timer...",
                        "Pomodoro cycle completed."
                    ]
                    # Pattern to extract message from rospy.loginfo lines
                    self.log_pattern = re.compile(r'\[INFO\] \[.*\]: (.+)')
                    self.buffer = ""

                def write(self, text):
                    # Accumulate text until a newline is encountered
                    self.buffer += text
                    while "\n" in self.buffer:
                        line, self.buffer = self.buffer.split("\n", 1)
                        line = line.strip()
                        # Check for "Alex:" messages
                        if self.keep_pattern_alex.search(line):
                            self._original_stdout.write(line + "\n")
                        # Check for the "User:" prompt
                        elif self.keep_pattern_user.match(line):
                            self._original_stdout.write("User: ")
                        # Check for allowed ROS log messages
                        else:
                            log_match = self.log_pattern.match(line)
                            if log_match:
                                message = log_match.group(1)
                                if message in self.allowed_log_messages:
                                    self._original_stdout.write(message + "\n")
                        # Ignore all other lines

                def flush(self):
                    # Handle any remaining buffer content
                    if self.buffer:
                        line = self.buffer.strip()
                        if self.keep_pattern_alex.search(line):
                            self._original_stdout.write(line + "\n")
                        elif self.keep_pattern_user.match(line):
                            self._original_stdout.write("User: ")
                        else:
                            log_match = self.log_pattern.match(line)
                            if log_match:
                                message = log_match.group(1)
                                if message in self.allowed_log_messages:
                                    self._original_stdout.write(message + "\n")
                        self.buffer = ""
                    self._original_stdout.flush()

            # Redirect sys.stdout through the custom TerminalFilter
            original_stdout = sys.stdout
            sys.stdout = TerminalFilter(original_stdout)

            # Initialize a ROS node for the combined application
            rospy.init_node('combined_app_node')

            # Create instances of both main applications
            engagement_app = MainApp()
            emotion_monitor = EmotionMonitor()

            # Run each application in a separate thread
            engagement_thread = threading.Thread(target=engagement_app.run)
            emotion_thread = threading.Thread(target=emotion_monitor.run)

            engagement_thread.start()
            emotion_thread.start()

            # Keep the main thread alive until ROS is shutdown
            try:
                while not rospy.is_shutdown():
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            finally:
                # Restore the original stdout to ensure normal printing after shutdown
                sys.stdout.flush()
                sys.stdout = original_stdout

        else:
            print("You must complete step 2 before this step.")


    elif choice.lower() == 'menu':
        print_menu()
    elif choice.lower() == 'exit':
        return False
    else:
        print("Invalid choice. Please try again. Enter 'menu' to see the options.")
    return True


# Main program loop handling login and menu execution
def main():
    print_intro()
    logged_in = False

    # Login/signup loop
    while (logged_in == False):
        print("Press 1 to log in")
        print("Press 2 to sign up")
        choice = input("Enter your choice: ")
        if (choice == '1'):
            logged_in = login()
        elif (choice == '2'):
            signup()
        else: 
            print("Invalid choice. Please try again.")
        print_divider()

    check_returning_user()

    print_menu()

    # Main program loop
    continue_program = True
    while continue_program:
        print("Enter 'menu' to see the options again")
        choice = input("Enter your choice: ")
        print_divider()
        continue_program = execute_menu(choice)
        print_divider()

if __name__ == "__main__":
    main()
