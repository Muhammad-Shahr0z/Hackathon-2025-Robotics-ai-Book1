"""
Test script to verify Better Auth functionality
"""
import asyncio
import aiohttp
import json

async def test_auth_endpoints():
    base_url = "http://localhost:8000"

    async with aiohttp.ClientSession() as session:
        print("Testing Better Auth endpoints...")

        # Test sign up
        import time
        unique_id = str(int(time.time() * 1000))  # Use timestamp for uniqueness
        print("\n1. Testing sign up...")
        signup_data = {
            "email": f"test_{unique_id}@example.com",
            "password": "password123",
            "name": "Test User",
            "softwareExperience": "intermediate",
            "hardwareExperience": "hobbyist",
            "programmingLanguages": ["Python", "JavaScript"],
            "roboticsBackground": "courses",
            "learningGoals": "Learn robotics"
        }

        try:
            async with session.post(f"{base_url}/api/auth/sign-up/email", json=signup_data) as resp:
                print(f"Sign up status: {resp.status}")
                response_data = await resp.json()
                print(f"Sign up response: {json.dumps(response_data, indent=2)}")

                if resp.status == 200:
                    print("Sign up successful!")
                    token = response_data.get('session', {}).get('token')
                else:
                    print("Sign up failed!")
                    return
        except Exception as e:
            print(f"Error during sign up: {e}")
            return

        # Test sign in
        print("\n2. Testing sign in...")
        signin_data = {
            "email": f"test_{unique_id}@example.com",  # Use the same unique email
            "password": "password123"
        }

        try:
            async with session.post(f"{base_url}/api/auth/sign-in/email", json=signin_data) as resp:
                print(f"Sign in status: {resp.status}")
                response_data = await resp.json()
                print(f"Sign in response: {json.dumps(response_data, indent=2)}")

                if resp.status == 200:
                    print("Sign in successful!")
                    token = response_data.get('session', {}).get('token')
                else:
                    print("Sign in failed!")
                    return
        except Exception as e:
            print(f"Error during sign in: {e}")
            return

        # Test get session with token
        print("\n3. Testing get session...")
        headers = {"Authorization": f"Bearer {token}"}

        try:
            async with session.get(f"{base_url}/api/auth/get-session", headers=headers) as resp:
                print(f"Get session status: {resp.status}")
                response_data = await resp.json()
                print(f"Get session response: {json.dumps(response_data, indent=2)}")

                if resp.status == 200 and response_data.get('user'):
                    print("Get session successful!")
                else:
                    print("Get session failed!")
        except Exception as e:
            print(f"Error during get session: {e}")

        # Test request password reset
        print("\n4. Testing request password reset...")
        reset_data = {
            "email": f"test_{unique_id}@example.com"  # Use the same unique email
        }

        try:
            async with session.post(f"{base_url}/api/auth/request-password-reset", json=reset_data) as resp:
                print(f"Request password reset status: {resp.status}")
                response_data = await resp.json()
                print(f"Request password reset response: {json.dumps(response_data, indent=2)}")

                if resp.status == 200:
                    print("Request password reset successful!")
                else:
                    print("Request password reset failed!")
        except Exception as e:
            print(f"Error during request password reset: {e}")

        print("\nAll tests completed!")

if __name__ == "__main__":
    asyncio.run(test_auth_endpoints())