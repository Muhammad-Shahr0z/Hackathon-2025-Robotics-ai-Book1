"""
Password Reset Token model for the Better Auth implementation.
This file defines the Password Reset Token entity based on the data-model.md specification.
"""
from typing import Optional
from datetime import datetime
from dataclasses import dataclass


@dataclass
class PasswordResetToken:
    """
    Represents a temporary token used to verify user identity during password recovery
    """
    id: str  # Unique token identifier
    user_id: str  # Reference to the user
    token_hash: str  # Securely hashed reset token
    expires_at: datetime  # Token expiration time
    used: bool  # Whether token has been used
    created_at: datetime  # Token creation time

    @classmethod
    def from_db_record(cls, record):
        """
        Create a PasswordResetToken instance from a database record
        """
        return cls(
            id=record['id'],
            user_id=record['user_id'],
            token_hash=record['token_hash'],
            expires_at=record['expires_at'],
            used=record.get('used', False),
            created_at=record['created_at']
        )

    def to_dict(self):
        """
        Convert the PasswordResetToken instance to a dictionary
        """
        return {
            'id': self.id,
            'user_id': self.user_id,
            'expires_at': self.expires_at.isoformat() if self.expires_at else None,
            'used': self.used,
            'created_at': self.created_at.isoformat() if self.created_at else None
        }