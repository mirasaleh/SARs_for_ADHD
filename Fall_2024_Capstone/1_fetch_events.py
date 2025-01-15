# Import required libraries for datetime operations and file handling
import datetime
import os.path
import csv

# Import Google Calendar API related libraries
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# Define the OAuth 2.0 scopes needed for Google Calendar API
# Warning: Modifying these scopes requires deletion of token.json
SCOPES = ["https://www.googleapis.com/auth/calendar"]

# Main function to fetch events from Google Calendar and save them to CSV.
# Handles authentication, API calls, and data processing.
def main():
    # Initialize credentials object
    creds = None
    
    # Load existing credentials from token.json if available
    if os.path.exists("token.json"):
        creds = Credentials.from_authorized_user_file("token.json", SCOPES)
    
    # Handle credential validation and refresh
    if not creds or not creds.valid:
        # Refresh credentials if expired but refresh token exists
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            # Create new authentication flow if no valid credentials exist
            flow = InstalledAppFlow.from_client_secrets_file(
                "credentials.json", SCOPES
            )
            creds = flow.run_local_server(port=0)
        # Save the credentials for future use
        with open("token.json", "w") as token:
            token.write(creds.to_json())

    try:
        # Build the Google Calendar API service
        service = build("calendar", "v3", credentials=creds)

        # Calculate time range for event fetch (next 7 days)
        now = datetime.datetime.utcnow()
        seven_days_later = now + datetime.timedelta(days=7)
        # Convert to ISO format with UTC indicator
        now = now.isoformat() + "Z"  # 'Z' indicates UTC time
        seven_days_later = seven_days_later.isoformat() + "Z"

        # Fetch events from primary calendar
        events_result = service.events().list(
            calendarId='primary',
            timeMin=now,
            timeMax=seven_days_later,
            singleEvents=True, # Expand recurring events
            orderBy='startTime' # Sort by start time
        ).execute()

        # Extract events and filter out all-day events
        events = events_result.get('items', [])
        filtered_events = [
            event for event in events 
            if 'dateTime' in event['start'] # Only include events with specific times
        ]

        # Handle case when no events are found
        if not filtered_events:
            print('No events found in the next 7 days.')
            return

        # Export events to CSV file
        with open('1_calendar_items.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            # Write CSV header
            csvwriter.writerow(['eventName', 'startDate', 'startTime', 'endDate', 'endTime'])

            # Process each event
            for event in filtered_events:
                # Extract event details with fallback for missing title
                summary = event.get('summary', 'No Title')
                start = event['start']['dateTime']
                end = event['end']['dateTime']

                # Convert ISO format to datetime objects
                start_datetime = datetime.datetime.fromisoformat(start.replace('Z', '+00:00'))
                end_datetime = datetime.datetime.fromisoformat(end.replace('Z', '+00:00'))

                # Format datetime components for CSV
                start_date = start_datetime.strftime('%Y-%m-%d')
                start_time = start_datetime.strftime('%H:%M:%S')
                end_date = end_datetime.strftime('%Y-%m-%d')
                end_time = end_datetime.strftime('%H:%M:%S')

                # Write event data to CSV
                csvwriter.writerow([summary, start_date, start_time, end_date, end_time])

        print("Events successfully fetched.")

    except HttpError as error:
        # Handle API errors
        print(f"An error occurred: {error}")

if __name__ == "__main__":
    main()
