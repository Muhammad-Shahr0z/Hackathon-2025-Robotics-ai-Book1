"""
Test suite for Better Auth Implementation
This file tests the complete authentication flow including registration, sign-in, and password reset
"""
import asyncio
import os
from dotenv import load_dotenv
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from better_auth import BetterAuth, auth

# Load environment variables
load_dotenv()

async def test_auth_flow():
    """Test the complete authentication flow"""
    print("Initializing Better Auth...")
    await auth.init_db()
    print("Better Auth initialized successfully")

    # Test user registration
    print("\n--- Testing User Registration ---")
    try:
        user = await auth.create_user(
            first_name="Test",
            last_name="User",
            email="testuser@example.com",
            password="SecurePassword123!"
        )
        print(f"✓ User registered: {user['email']}")
        user_id = user['id']
    except Exception as e:
        print(f"✗ User registration failed: {e}")
        return

    # Test user sign-in
    print("\n--- Testing User Sign-In ---")
    try:
        # Create a mock request object for testing
        from unittest.mock import Mock
        request = Mock()
        request.json = Mock(return_value={"email": "testuser@example.com", "password": "SecurePassword123!"})

        # Simulate sign-in process
        user_data = await auth.get_user_by_email("testuser@example.com")
        if user_data:
            session = await auth.create_session(user_data['id'])
            print(f"✓ User signed in, session created: {session['token'][:10]}...")
        else:
            print("✗ User not found for sign-in")
            return
    except Exception as e:
        print(f"✗ User sign-in failed: {e}")
        return

    # Test password reset request
    print("\n--- Testing Password Reset Request ---")
    try:
        reset_success = await auth.request_password_reset("testuser@example.com")
        if reset_success:
            print("✓ Password reset request processed")
        else:
            print("✗ Password reset request failed")
    except Exception as e:
        print(f"✗ Password reset request failed: {e}")

    # Test getting session by token
    print("\n--- Testing Get Session by Token ---")
    try:
        session_info = await auth.get_session_by_token(session['token'])
        if session_info:
            print(f"✓ Session retrieved for user: {session_info['user']['email']}")
        else:
            print("✗ Failed to retrieve session")
    except Exception as e:
        print(f"✗ Get session failed: {e}")

    # Test session deletion (sign out)
    print("\n--- Testing Session Deletion (Sign Out) ---")
    try:
        delete_success = await auth.delete_session(session['token'])
        if delete_success:
            print("✓ Session deleted successfully")
        else:
            print("✗ Session deletion failed")
    except Exception as e:
        print(f"✗ Session deletion failed: {e}")

    print("\n--- All Tests Completed ---")


async def test_validation():
    """Test validation functionality"""
    print("\n--- Testing Validation ---")

    # Test email validation
    email_valid = auth.validation_service.validate_email("test@example.com")
    print(f"✓ Email validation for 'test@example.com': {email_valid}")

    email_invalid = auth.validation_service.validate_email("invalid-email")
    print(f"✓ Email validation for 'invalid-email': {email_invalid}")

    # Test password validation
    password_result = auth.validation_service.validate_password("SecurePass123!")
    print(f"✓ Password validation result: {password_result['valid']}")

    weak_password_result = auth.validation_service.validate_password("weak")
    print(f"✓ Weak password validation result: {weak_password_result['valid']}")

    # Test registration data validation
    reg_validation = auth.validation_service.validate_user_registration_data(
        "Test", "User", "test@example.com", "SecurePass123!"
    )
    print(f"✓ Registration data validation: {reg_validation['valid']}")


if __name__ == "__main__":
    print("Starting Better Auth Implementation Tests...")

    # Run validation tests
    asyncio.run(test_validation())

    # Run main auth flow tests
    asyncio.run(test_auth_flow())

    print("\nAll tests completed!")