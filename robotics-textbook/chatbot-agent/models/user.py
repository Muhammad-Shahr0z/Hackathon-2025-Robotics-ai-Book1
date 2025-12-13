"""
User model for the Better Auth implementation.
This file defines the User entity based on the data-model.md specification.
"""
from typing import Optional, List
from datetime import datetime
from dataclasses import dataclass


@dataclass
class User:
    """
    Represents a registered user in the system
    """
    id: str  # Unique identifier for the user
    first_name: str  # User's first name
    last_name: str  # User's last name
    email: str  # User's email address (unique, validated)
    password_hash: str  # Securely hashed password
    created_at: datetime  # Account creation timestamp
    updated_at: datetime  # Last update timestamp
    email_verified: bool = False  # Whether email has been verified
    active: bool = True  # Whether account is active

    @classmethod
    def from_db_record(cls, record):
        """
        Create a User instance from a database record
        """
        return cls(
            id=record['id'],
            first_name=record.get('first_name', ''),
            last_name=record.get('last_name', ''),
            email=record['email'],
            password_hash=record['password_hash'],
            created_at=record['created_at'],
            updated_at=record['updated_at'],
            email_verified=record.get('email_verified', False),
            active=record.get('active', True)
        )

    def to_dict(self):
        """
        Convert the User instance to a dictionary
        """
        return {
            'id': self.id,
            'first_name': self.first_name,
            'last_name': self.last_name,
            'email': self.email,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'updated_at': self.updated_at.isoformat() if self.updated_at else None,
            'email_verified': self.email_verified,
            'active': self.active
        }