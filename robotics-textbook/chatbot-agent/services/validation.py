"""
Validation Service for the Better Auth implementation.
This file implements validation logic for authentication functionality.
"""
import re
from typing import Optional, Dict, Any


class ValidationService:
    """
    Validation Service for authentication data validation
    """

    @staticmethod
    def validate_email(email: str) -> bool:
        """
        Validate email format using regex
        """
        if not email or not isinstance(email, str):
            return False

        # Basic email regex pattern
        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(pattern, email) is not None

    @staticmethod
    def validate_password(password: str) -> Dict[str, Any]:
        """
        Validate password strength
        Returns a dictionary with 'valid' (bool) and 'errors' (list)
        """
        errors = []

        if not password:
            errors.append("Password is required")
            return {"valid": False, "errors": errors}

        if len(password) < 6:
            errors.append("Password must be at least 6 characters long")

        return {
            "valid": len(errors) == 0,
            "errors": errors
        }

    @staticmethod
    def validate_user_registration_data(first_name: str, last_name: str, email: str, password: str) -> Dict[str, Any]:
        """
        Validate user registration data
        Returns a dictionary with 'valid' (bool) and 'errors' (dict with field-specific errors)
        """
        errors = {}

        # Validate email
        if not email:
            errors['email'] = ["Email is required"]
        elif not ValidationService.validate_email(email):
            errors['email'] = ["Invalid email format"]

        # Validate password
        password_validation = ValidationService.validate_password(password)
        if not password_validation['valid']:
            errors['password'] = password_validation['errors']

        return {
            "valid": len(errors) == 0,
            "errors": errors
        }

    @staticmethod
    def validate_user_signin_data(email: str, password: str) -> Dict[str, Any]:
        """
        Validate user sign-in data
        Returns a dictionary with 'valid' (bool) and 'errors' (dict with field-specific errors)
        """
        errors = {}

        # Validate email
        if not email:
            errors['email'] = ["Email is required"]
        elif not ValidationService.validate_email(email):
            errors['email'] = ["Invalid email format"]

        # Validate password
        if not password:
            errors['password'] = ["Password is required"]

        return {
            "valid": len(errors) == 0,
            "errors": errors
        }

    @staticmethod
    def validate_password_reset_data(token: str, new_password: str) -> Dict[str, Any]:
        """
        Validate password reset data
        Returns a dictionary with 'valid' (bool) and 'errors' (dict with field-specific errors)
        """
        errors = {}

        # Validate token
        if not token or len(token.strip()) == 0:
            errors['token'] = ["Token is required"]

        # Validate new password
        password_validation = ValidationService.validate_password(new_password)
        if not password_validation['valid']:
            errors['new_password'] = password_validation['errors']

        return {
            "valid": len(errors) == 0,
            "errors": errors
        }