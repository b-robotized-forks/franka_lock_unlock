# Copyright jk-ethz
# Released under GNU AGPL-3.0
# Contact us for other licensing options.

# Developed and tested on system version
# 4.2.1

# Inspired by
# https://github.com/frankaemika/libfranka/issues/63
# https://github.com/ib101/DVK/blob/master/Code/DVK.py

from abc import ABC, abstractmethod
import hashlib
import base64
import requests
from urllib.parse import urljoin
from http import HTTPStatus
from time import sleep

SELF_TEST_WAIT = 120 # seconds

class FrankaClient(ABC):
    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        protocol: str = "https",
        timeout: float = 5.0,
    ):
        requests.packages.urllib3.disable_warnings()
        self._session = requests.Session()
        self._session.verify = False
        self._hostname = f"{protocol}://{hostname}"
        self._username = username
        self._password = password
        self._logged_in = False
        self._token = None
        self._token_id = None
        self.timeout = timeout

    @staticmethod
    def _encode_password(username, password):
        bs = ",".join(
            [str(b) for b in hashlib.sha256((f"{password}#{username}@franka").encode()).digest()]
        )
        return base64.encodebytes(bs.encode("utf-8")).decode("utf-8")

    def _login(self):
        print("Logging in...")
        if self._logged_in:
            print("Already logged in.")
            return
        login = self._session.post(
            urljoin(self._hostname, "/admin/api/login"),
            json={
                "login": self._username,
                "password": self._encode_password(self._username, self._password),
            },
            timeout=self.timeout,
        )
        assert login.status_code == HTTPStatus.OK, "Error logging in."
        self._session.cookies.set("authorization", login.text)
        self._logged_in = True
        print("Successfully logged in.")

    def _logout(self):
        print("Logging out...")
        assert self._logged_in
        logout = self._session.post(urljoin(self._hostname, "/admin/api/logout"))
        assert logout.status_code == HTTPStatus.OK, "Error logging out"
        self._session.cookies.clear()
        self._logged_in = False
        print("Successfully logged out.")

    def _shutdown(self):
        print("Shutting down...")
        assert self._is_active_token(), "Cannot shutdown without an active control token."
        try:
            self._session.post(
                urljoin(self._hostname, "/admin/api/shutdown"), json={"token": self._token}
            )
        except requests.exceptions.RequestException as e:
            # Sometimes, the server can shut down before sending a complete response, possibly raising an exception.
            # Anyways, the server has still received the request, thus the robot shutdown procedure will start.
            # So, we can ignore the cases when these exceptions are raised.
            print(f"Request exception with error: {e}")
        finally:
            print(
                "The robot is shutting down. Please wait for the yellow lights to turn off, then switch the control box off."
            )

    def _get_active_token_id(self):
        token_query = self._session.get(urljoin(self._hostname, "/admin/api/control-token"))
        assert token_query.status_code == HTTPStatus.OK, "Error getting control token status."
        json = token_query.json()
        return None if json["activeToken"] is None else json["activeToken"]["id"]

    def _is_active_token(self):
        active_token_id = self._get_active_token_id()
        return active_token_id is None or active_token_id == self._token_id

    def _request_token(self, physically=False):
        print("Requesting a control token...")
        if self._token is not None:
            assert self._token_id is not None
            print("Already having a control token.")
            return
        token_request = self._session.post(
            urljoin(
                self._hostname, f'/admin/api/control-token/request{"?force" if physically else ""}'
            ),
            json={"requestedBy": self._username},
        )
        assert token_request.status_code == HTTPStatus.OK, "Error requesting control token."
        json = token_request.json()
        self._token = json["token"]
        self._token_id = json["id"]
        print(f"Received control token is {self._token} with id {self._token_id}.")

    def _release_token(self):
        print("Releasing control token...")
        token_delete = self._session.delete(
            urljoin(self._hostname, "/admin/api/control-token"), json={"token": self._token}
        )
        assert token_delete.status_code == 200, "Error releasing control token."
        self._token = None
        self._token_id = None
        print("Successfully released control token.")

    @abstractmethod
    def run(self) -> None:
        pass

    def _acknowledge_self_test_error(self, error_id: str = "TD2Timeout") -> tuple[bool, str]:
        """Acknolwedges the self test error if occured."""
        headers = {"X-Control-Token": self._token}

        # Acknowledge the error (if required)
        ack_url = urljoin(
            self._hostname,
            f"/admin/api/safety/recoverable-safety-errors/acknowledge?error_id={error_id}",
        )
        ack_res = self._session.post(ack_url, headers=headers, timeout=self.timeout)

        if ack_res.status_code in (200, 204):
            return True, f"Successfully acknowledged error '{error_id}'."
        elif (
            ack_res.status_code == 424 and "NoAckRequired" in ack_res.text
        ) or ack_res.status_code == 404:
            return True,  "No acknowledgment required (already acknowledged or not pending). Proceeding to execution..."
        else:
            return False, f"Acknowledgment returned {ack_res.status_code}: {ack_res.text}. Attempting execution anyway..."

    def _trigger_self_test(self) -> tuple[bool, str]:
        """Triggers the Self Test."""
        headers = {"X-Control-Token": self._token}
        exec_url = urljoin(self._hostname, "/admin/api/safety/td2-tests/execute")
        print("Executing TD2 self-test...")
        exec_res = self._session.post(exec_url, headers=headers, timeout=self.timeout)

        if exec_res.status_code not in (200, 204):
            # 424 ActionUnavailable can mean no test is pending/needed right now
            if exec_res.status_code == 424:
                return True, f"TD2 execute unavailable ({exec_res.text}). System may already be operational."
            return False, f"Failed to execute TD2 test: {exec_res.status_code} - {exec_res.text}"
        return True, "TD2 self-test triggered successfully. Waiting for completion..."

    def _wait_until_self_test_completes(self) -> tuple[bool, str]:
        """Waits until the self test completes."""
        # NOTE: Currently there is no API to check the status of the Franka Robot to check if self-test is completed.
        # Hence, we wait 2 mins which is tested on the FR3.
        #
        sleep(SELF_TEST_WAIT)
        return True, "Self test completed."
