#!/usr/bin/env python3
"""
Deployment script to interface with Vercel MCP server for deploying the FastAPI backend.
This script uses the Vercel CLI to deploy the application to Vercel.
"""

import os
import subprocess
import sys
from pathlib import Path


def check_vercel_cli():
    """Check if Vercel CLI is installed."""
    try:
        result = subprocess.run(['vercel', '--version'],
                                capture_output=True, text=True, check=True)
        print(f"Vercel CLI version: {result.stdout.strip()}")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Vercel CLI not found. Please install it with: npm install -g vercel")
        return False


def login_to_vercel():
    """Login to Vercel if not already logged in."""
    try:
        result = subprocess.run(['vercel', 'whoami'],
                                capture_output=True, text=True, check=True)
        print(f"Already logged in to Vercel as: {result.stdout.strip()}")
        return True
    except subprocess.CalledProcessError:
        print("Not logged in to Vercel. Initiating login...")
        try:
            subprocess.run(['vercel', 'login'], check=True)
            return True
        except subprocess.CalledProcessError:
            print("Failed to login to Vercel.")
            return False


def deploy_to_vercel():
    """Deploy the application to Vercel."""
    try:
        # Change to the project root directory
        project_root = Path(__file__).parent
        os.chdir(project_root)

        print("Deploying to Vercel...")
        # Use the vercel.json configuration file
        result = subprocess.run([
            'vercel',
            '--local-config=vercel.json',  # Use our custom config
            '--public',  # Deploy as public project
            '--yes'      # Skip prompts
        ], check=True, capture_output=True, text=True)

        print("Deployment successful!")
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Deployment failed: {e}")
        print(f"Error output: {e.stderr}")
        return False


def main():
    """Main deployment function."""
    print("Starting deployment to Vercel...")

    if not check_vercel_cli():
        print("Please install Vercel CLI and try again.")
        sys.exit(1)

    if not login_to_vercel():
        print("Please login to Vercel and try again.")
        sys.exit(1)

    if deploy_to_vercel():
        print("Deployment completed successfully!")
    else:
        print("Deployment failed. Please check the error messages above.")
        sys.exit(1)


if __name__ == "__main__":
    main()