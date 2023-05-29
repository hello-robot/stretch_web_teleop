# Firebase
Firebase is a set of application development platforms and backen cloud computing services. We will be using Firebase's Realtime Database for data storage.

## Setting up Firebase
### Creating a Firebase Project
Sign into [Firebase](https://firebase.google.com/) with your google account then open the Firebase [console](https://console.firebase.google.com/) and create a new project. The project will default to using the no-cost [Spark plan](https://firebase.google.com/pricing?hl=en&authuser=1). 

Add a web app to your firebase project. You shouldn't need to worry about installing the Firebase SDK because it is already in the `package.json` dependencies for this repo. This will generate a configuration for your web app that looks something like this:
```
const firebaseConfig = {
  apiKey: ...,
  authDomain: ...,
  projectId: ...,
  storageBucket: ...,
  messagingSenderId: ...,
  appId: ...,
  measurementId: ...
};
```

Create a file named `.env` in `stretch-web-interface` and add the config to the `.env` file. The config will need to be reformatted slightly so the contents of `.env` look like this:
```
apiKey=DEzaSyAzZEQ89KBuKXgKJ-UWV9vm3xM
authDomain=stretch-teleop-interface.firebaseapp.com
projectId=stretch-teleop-interface
storageBucket=stretch-teleop-interface.appspot.com
messagingSenderId=124457856584
appId=1:364440456284:web:1e2603a456f839280det99
measurementId=T-6GMDF5W03Z
```

### Setup the Realtime Database
Select the `Realtime Database` option under Build in the Firebase console for your project, then create a database. Select "Start in **locked mode**" in `Security Rules` and click `Enable`. Add the following to the database rules:
```
{
    "rules": {
        ".write": "auth.token.email == 'user1@example.com' || auth.token.email == 'user2@example.com'",
        "users" : {
	        "$user_id" : {
            ".write": "$user_id === auth.uid",
            ".read": "$user_id === auth.uid"
          }
        },
        "layouts": {
        	".read": "auth != null"
        },
        "currentLayouts": {
        	".read": "auth != null"
        }
    }
}
```

Replace `'user1@example.com'` and `'user2@example.com'` with the email addresses of the users you'd like to give write access to. You can add as many users as you'd like by separating them with `||`.

### Setup Authentication
Select the `Authentication` option under Build in the Firebase console for your project, then click `Get Started`. Click `Email/Password` and enable it. Do not enable passwordless sign-in. Click `Add new Provider` and `Anonymous` then enable it and click `Save`. Finally, add another provider, click `Google` and add a `Project public-facing name`, select a support email and click `Save`. We will primarily be using `Google` for authentication.

