# Data Model: Better Auth Implementation

## Entity: User
**Description**: Represents a registered user in the system

**Fields**:
- `id` (string/UUID): Unique identifier for the user
- `first_name` (string): User's first name
- `last_name` (string): User's last name
- `email` (string): User's email address (unique, validated)
- `password_hash` (string): Securely hashed password
- `created_at` (timestamp): Account creation timestamp
- `updated_at` (timestamp): Last update timestamp
- `email_verified` (boolean): Whether email has been verified
- `active` (boolean): Whether account is active

**Validation Rules**:
- Email must be valid email format
- Email must be unique
- First name and last name must not be empty
- Password must meet security requirements

## Entity: Authentication Session
**Description**: Represents an active authenticated session for a user

**Fields**:
- `id` (string/UUID): Unique session identifier
- `user_id` (string): Reference to the user
- `session_token` (string): Secure session token
- `expires_at` (timestamp): Session expiration time
- `created_at` (timestamp): Session creation time
- `last_accessed` (timestamp): Last access time
- `device_info` (string): Information about the device

**Validation Rules**:
- Session must be linked to an active user
- Session must not be expired
- Token must be securely generated

## Entity: Password Reset Token
**Description**: Represents a temporary token used to verify user identity during password recovery

**Fields**:
- `id` (string/UUID): Unique token identifier
- `user_id` (string): Reference to the user
- `token_hash` (string): Securely hashed reset token
- `expires_at` (timestamp): Token expiration time
- `used` (boolean): Whether token has been used
- `created_at` (timestamp): Token creation time

**Validation Rules**:
- Token must be linked to an active user
- Token must not be expired
- Token can only be used once
- Token must be securely generated