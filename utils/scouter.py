import requests
import csv
import time

# Event info
SEASON = 0 # this will  be overridden
EVENT_CODE = "" # this will be overridden
BASE_URL = "https://api.ftcscout.org/rest/v1"

def get_event_teams(season, event_code):
    """
    Get all teams for the event
    """
    url = f"{BASE_URL}/events/{season}/{event_code}/teams"
    resp = requests.get(url)
    resp.raise_for_status()
    return resp.json()

def get_team_stats(team_number, season):
    """
    Get quick stats for a team
    """
    url = f"{BASE_URL}/teams/{team_number}/quick-stats?season={season}"
    resp = requests.get(url)
    if resp.status_code == 404:
        # no data for this team in that season/event
        return None
    resp.raise_for_status()
    return resp.json()

def get_team_name(team_number):
    url = f"{BASE_URL}/teams/{team_number}"
    resp = requests.get(url)
    resp.raise_for_status()
    return resp.json().get("name")

def main():
    SEASON = int(input("enter the desired season: "))
    EVENT_CODE = input("enter the event code (ex: USTXCMP): ")

    print(f"Fetching teams for event {EVENT_CODE} (Season {SEASON})…")
    event_data = get_event_teams(SEASON, EVENT_CODE)

    # event_data is a list of team event participations
    teams = event_data

    output_rows = []

    for team_participation in teams:
        team_number = team_participation["teamNumber"]
        team_name = get_team_name(team_number)
        print(f"Getting stats for team {team_number}: {team_name}")

        stats = get_team_stats(team_number, SEASON)
        if not stats:
            print(f"  ⚠️ No quick-stats found for {team_number}")
            continue

        # `stats` quick-stats fields. API returns `tot`, `auto`, `dc`, `eg`.
        best_np_opr = stats.get("tot", {}).get("value")
        auto_opr = stats.get("auto", {}).get("value")
        teleop_opr = stats.get("dc", {}).get("value") # driver controlled i.e TeleOp
        endgame_opr = stats.get("eg", {}).get("value") # eg = endgame

        output_rows.append([
            team_number,
            team_name,
            best_np_opr,
            auto_opr,
            teleop_opr,
            endgame_opr,
        ])

    # Write CSV
    FILENAME = f"{SEASON}_{EVENT_CODE}.csv"
    with open(FILENAME, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Team Number",
            "Team Name",
            "Best NP OPR",
            "Auto OPR",
            "TeleOp OPR",
            "Endgame OPR"
        ])
        writer.writerows(output_rows)

    print(f"Done! CSV saved to {FILENAME}")

if __name__ == "__main__":
    main()
