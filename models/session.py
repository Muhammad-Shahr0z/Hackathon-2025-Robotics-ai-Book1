"""
Authentication Session model for the Better Auth implementation.
This file defines the Authentication Session entity based on the data-model.md specification.
"""
from typing import Optional
from datetime import datetime
from dataclasses import dataclass


@dataclass
class AuthSession:
    """
    Represents an active authenticated session for a user
    """
    id: str  # Unique session identifier
    user_id: str  # Reference to the user
    session_token: str  # Secure session token
    expires_at: datetime  # Session expiration time
    created_at: datetime  # Session creation time
    last_accessed: Optional[datetime] = None  # Last access time
    device_info: Optional[str] = None  # Information about the device

    @classmethod
    def from_db_record(cls, record):
        """
        Create an AuthSession instance from a database record
        """
        return cls(
            id=record['id'],
            user_id=record['user_id'],
            session_token=record['token'],
            expires_at=record['expires_at'],
            created_at=record['created_at'],
            last_accessed=record.get('last_accessed'),
            device_info=record.get('device_info')
        )

    def to_dict(self):
        """
        Convert the AuthSession instance to a dictionary
        """
        return {
            'id': self.id,
            'user_id': self.user_id,
            'session_token': self.session_token,
            'expires_at': self.expires_at.isoformat() if self.expires_at else None,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'last_accessed': self.last_accessed.isoformat() if self.last_accessed else None,
            'device_info': self.device_info
        }