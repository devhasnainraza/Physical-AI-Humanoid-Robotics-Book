FROM node:20-alpine

WORKDIR /app

COPY package.json package-lock.json ./

RUN npm ci

COPY . .

# Build the Docusaurus site
RUN npm run build

# Expose port 3000
EXPOSE 3000

# Serve the production build
CMD ["npm", "run", "serve", "--", "--host", "0.0.0.0", "--no-open"]
