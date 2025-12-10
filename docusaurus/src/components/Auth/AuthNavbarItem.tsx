import React, { JSX, useEffect, useState } from 'react';
import { translate } from '@docusaurus/Translate';
import Link from '@docusaurus/Link';
import { logout, getAccessToken } from '@site/src/lib/authAdapter';

interface AuthNavbarItemProps {
  position?: 'left' | 'right';
}

export default function AuthNavbarItem({ position }: AuthNavbarItemProps): JSX.Element {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [userEmail, setUserEmail] = useState('');
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check if user is logged in by checking for access token
    const token = getAccessToken();
    setIsLoggedIn(!!token);

    // For a complete implementation, we would fetch user details from the backend
    // For now, we'll just extract email from localStorage if available
    if (token) {
      // In a real implementation, you would decode the JWT to get user info
      // or make an API call to get user details
      // For this example, we'll use a placeholder email
      setUserEmail('user@example.com');
    }

    setIsLoading(false);

    // Set up event listener to update state when auth changes
    const handleStorageChange = () => {
      const newToken = getAccessToken();
      setIsLoggedIn(!!newToken);
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  const handleLogout = () => {
    logout();
    setIsLoggedIn(false);
    window.location.href = '/login';
  };

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = () => {
      setIsMenuOpen(false);
    };

    if (isMenuOpen) {
      document.addEventListener('click', handleClickOutside);
    }

    return () => {
      document.removeEventListener('click', handleClickOutside);
    };
  }, [isMenuOpen]);

  if (isLoading) {
    return (
      <div className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}>
        Loading...
      </div>
    );
  }

  if (isLoggedIn) {
    // User is authenticated
    return (
      <div className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}>
        <div className="dropdown dropdown--hoverable dropdown--right">
          <a
            href="#"
            className="navbar__link"
            onClick={(e) => {
              e.preventDefault();
              setIsMenuOpen(!isMenuOpen);
            }}
          >
            👤 {userEmail.split('@')[0]}
          </a>

          {isMenuOpen && (
            <ul className="dropdown__menu">
              <li>
                <Link className="dropdown__link" to="/profile">
                  Your Profile
                </Link>
              </li>
              <li>
                <a
                  className="dropdown__link"
                  href="#"
                  onClick={(e) => {
                    e.preventDefault();
                    handleLogout();
                  }}
                >
                  Sign Out
                </a>
              </li>
            </ul>
          )}
        </div>
      </div>
    );
  } else {
    // User is not authenticated
    return (
      <div className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}>
        <Link to="/login" className="navbar__link">
          Sign In
        </Link>
      </div>
    );
  }
}