#!/usr/bin/env python3
# website_blocker.py

import sys
import os
import shutil

# Path to the system's hosts file
hosts_file_path = "/etc/hosts"  # For Linux/MacOS
# hosts_file_path = r"C:\Windows\System32\drivers\etc\hosts"  # For Windows

# The local address to redirect to (blocking the websites)
redirect = "127.0.0.1"

# List of websites to block
websites_to_block = [
    "www.netflix.com", "netflix.com", "netflix.com/", "www.netflix.com/"
    "www.instagram.com", "instagram.com", "i.instagram.com",
    "cdninstagram.com", "scontent.cdninstagram.com",
    "www.facebook.com", "facebook.com", "facebook.com/", "www.facebook.com/"
    "www.x.com", "x.com",
    "www.twitter.com", "twitter.com",
    "www.youtube.com", "youtube.com",
]

# Backup file path to store the original hosts file
hosts_backup_path = "/etc/hosts_backup"  # Ensure you have write access here

# Function to clear the DNS cache
def clear_dns_cache():
    print("Clearing DNS cache...")
    os.system("sudo systemd-resolve --flush-caches")  # For Linux with systemd
    # os.system("sudo dscacheutil -flushcache; sudo killall -HUP mDNSResponder")  # For macOS
    print("DNS cache cleared.")

# Function to create a backup of the original hosts file
def create_backup():
    if not os.path.exists(hosts_backup_path):
        print("Backing up the hosts file...")
        shutil.copyfile(hosts_file_path, hosts_backup_path)
        #print("Backup created.")
    else:
        #print("Backup already exists.")
	pass
# Function to restore the original hosts file from the backup
def restore_backup():
    if os.path.exists(hosts_backup_path):
        print("Restoring hosts file from backup...")
        shutil.copyfile(hosts_backup_path, hosts_file_path)
        clear_dns_cache()
        print("Hosts file restored.")
    else:
        print("No backup found. Skipping restore.")

# Function to block the websites
def block_websites():
    create_backup()
    print("Blocking websites...")
    with open(hosts_file_path, "r+") as hosts_file:
        hosts_content = hosts_file.read()
        for website in websites_to_block:
            entry = f"{redirect} {website}\n"
            if entry not in hosts_content:
                hosts_file.write(entry)
        hosts_file.flush()
    clear_dns_cache()
    print("Websites are blocked.")

# Function to unblock the websites
def unblock_websites():
    print("Unblocking websites...")
    restore_backup()
    print("Websites are unblocked.")

# Main function
def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['block', 'unblock']:
        print("Usage: website_blocker.py [block|unblock]")
        sys.exit(1)
    action = sys.argv[1]
    if action == 'block':
        block_websites()
    elif action == 'unblock':
        unblock_websites()

if __name__ == "__main__":
    main()

