# Import required libraries for datetime operations and API interaction
import datetime
import os.path
import pandas as pd
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# Define OAuth 2.0 scopes needed for Google Calendar API
SCOPES = ["https://www.googleapis.com/auth/calendar"]

# Fetch existing events from Google Calendar
def get_existing_events(service):
    # Fetch existing events from the calendar
    now = datetime.datetime.utcnow().isoformat() + 'Z'  # 'Z' indicates UTC time
    events_result = service.events().list(calendarId='primary', timeMin=now,
                                          maxResults=100, singleEvents=True,
                                          orderBy='startTime').execute()
    return {event['summary']: event for event in events_result.get('items', [])}

# Import schedule from CSV file and convert to Google Calendar event format
def import_schedule():
    # Read the CSV file
    df = pd.read_csv('5_block_on_cal.csv')
    events = []

    # Function to append ':00' to time if seconds are missing
    def ensure_seconds(time_str):
        if len(time_str.split(":")) == 2:  # Check if time is in HH:MM format
            return time_str + ":00"
        return time_str  # Already in HH:MM:SS format

    # Convert each row into the desired format
    for _, row in df.iterrows():
        try:
            start_time = ensure_seconds(row['startTime'])
            end_time = ensure_seconds(row['endTime'])
            
            event = {
                'summary': row['eventName'],
                'start': {
                    'dateTime': f"{row['startDate']}T{start_time}+04:00",
                    'timeZone': 'Asia/Dubai',
                },
                'end': {
                    'dateTime': f"{row['endDate']}T{end_time}+04:00",
                    'timeZone': 'Asia/Dubai',
                },
            }
            events.append(event)
        except KeyError as e:
            print(f"Missing column in row: {row}. Error: {e}")
        except Exception as e:
            print(f"Error processing row: {row}. Error: {e}")

    return events

# Main function to handle Google Calendar authentication and event creation
def main():
    # Initialize credentials
    creds = None
    if os.path.exists("token.json"):
        creds = Credentials.from_authorized_user_file("token.json", SCOPES)

    # Handle credential refresh or new authentication
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file("credentials.json", SCOPES)
            creds = flow.run_local_server(port=0)

        # Save credentials for future use
        with open("token.json", "w") as token:
            token.write(creds.to_json())

    try:
        # Build Google Calendar API service
        service = build("calendar", "v3", credentials=creds)
        events_to_add = import_schedule()

        # Get existing events to avoid duplicates
        existing_events = get_existing_events(service)

        # Add new events to calendar
        for event in events_to_add:
            try:
                if event['summary'] not in existing_events:
                    created_event = service.events().insert(calendarId='primary', body=event).execute()
                else:
                    print(f"Event '{event['summary']}' already exists and will not be added.")
            except HttpError as error:
                print(f"An error occurred for event {event}: {error}")
    except HttpError as error:
        print(f"An error occurred: {error}")

if __name__ == "__main__":
    main()
