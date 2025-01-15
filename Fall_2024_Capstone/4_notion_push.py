# Import required libraries for API interaction and data handling
import base64
import json
import os
import requests
import urllib.parse
from notion_client import Client
import datetime
import csv

# Notion API Authentication credentials
oauth_client_id = ""
oauth_client_secret = ""
redirect_uri = ""

# Create OAuth authentication URL
parsed_redirect_uri = urllib.parse.quote_plus(redirect_uri)
auth_url = f"https://api.notion.com/v1/oauth/authorize?client_id={oauth_client_id}&response_type=code&owner=user&redirect_uri={parsed_redirect_uri}"

# Authentication code obtained from redirect URL
auth_code = ""

# Encode client credentials for authentication header
key_secret = f'{oauth_client_id}:{oauth_client_secret}'.encode('ascii')
b64_encoded_key = base64.b64encode(key_secret).decode('ascii')

# Configure OAuth token request
base_url = 'https://api.notion.com/v1/oauth/token'
auth_headers = {
    'Authorization': f'Basic {b64_encoded_key}',
    'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8',
}

auth_data = {
    'grant_type': 'authorization_code',
    'code': auth_code,
    'redirect_uri': redirect_uri,
}

# Request access token
auth_resp = requests.post(base_url, headers=auth_headers, data=auth_data)
access_token = ""

# Initialize Notion client with access token
notion = Client(auth=access_token)

def read_weekly_tasks(file_path):
    tasks = {}
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            day, task = row
            if day not in tasks:
                tasks[day] = []
            tasks[day].append(task)
    return tasks

# Write tasks to Notion page organized by day
def write_tasks_to_notion(notion, page_id, tasks):
    for day, day_tasks in tasks.items():
        # Create a header for each day
        notion.blocks.children.append(
            block_id=page_id,
            children=[{
                "object": "block",
                "type": "heading_3",
                "heading_3": {
                    "rich_text": [{"type": "text", "text": {"content": day}}]
                }
            }]
        )
        
        # Add tasks for the day
        for task in day_tasks:
            notion.blocks.children.append(
                block_id=page_id,
                children=[{
                    "object": "block",
                    "type": "to_do",
                    "to_do": {
                        "rich_text": [{"type": "text", "text": {"content": task}}],
                        "checked": False
                    }
                }]
            )
    
    print("All tasks have been added to Notion.")

# Main function to handle task synchronization with Notion
def main():
    # Get page_id
    page_id = notion.search(query="to-do list").get("results")[0]["id"]

    # Read tasks from CSV
    tasks = read_weekly_tasks('3_weekly_tasks.csv')
    
    # Write tasks to Notion
    write_tasks_to_notion(notion, page_id, tasks)

if __name__ == "__main__":
    main()